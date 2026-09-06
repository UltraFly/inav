-- SPDX-License-Identifier: GPL-3.0-or-later
--
-- INAV Avian ESC telemetry and TextGen tool for EdgeTX color radios.
-- Version 0.3 is deliberately read-only on real aircraft. It displays
-- discovered telemetry and raw 4-in-1 TextGen, and provides a simulator UI
-- for interfaces that still require matching INAV MSP messages.

local APP_NAME = "INAV AVIAN ESC"
local APP_VERSION = "0.3.0"

local TEXTGEN_ID = 0x0C
local TEXTGEN_ROWS = 9
local TEXTGEN_COLS = 13
local TELEMETRY_STALE_TICKS = 300 -- getTime() uses 10 ms ticks.
local MSP2_INAV_STATUS = 0x2000
local MSP_STATUS_POLL_TICKS = 100
local ARMING_FLAG_ARMED = 0x04

local PAGE_HOME = 1
local PAGE_STATUS = 2
local PAGE_ESC = 3
local PAGE_TEXTGEN = 4

local pageItems = {
    { "Live status", PAGE_STATUS },
    { "Avian ESC telemetry", PAGE_ESC },
    { "Avian ESC TextGen", PAGE_TEXTGEN },
}

local pageTitles = {
    [PAGE_STATUS] = "LIVE STATUS",
    [PAGE_ESC] = "AVIAN ESC",
    [PAGE_TEXTGEN] = "AVIAN TEXTGEN",
}

local FLAG_SMALL = SMLSIZE or 0
local FLAG_MEDIUM = MIDSIZE or 0
local FLAG_BOLD = BOLD or 0
local FLAG_RIGHT = RIGHT or 0
local FLAG_CENTER = CENTER or 0
local FLAG_INVERSE = INVERS or 0
local FLAG_BLINK = BLINK or 0

local SCREEN_W = LCD_W or 480
local SCREEN_H = LCD_H or 272
local HEADER_H = 30
local FOOTER_H = 25

local function now()
    if type(getTime) == "function" then
        return getTime()
    end
    return 0
end

local function ticksSince(tick, previous)
    if tick >= previous then
        return tick - previous
    end
    return tick + 0x80000000 - previous
end

local function rounded(value)
    if value >= 0 then
        return math.floor(value + 0.5)
    end
    return math.ceil(value - 0.5)
end

local function eventIs(event, expected)
    return expected ~= nil and event == expected
end

local state = {
    page = PAGE_HOME,
    homeFocus = 1,
    simulator = false,
    notice = nil,
    noticeUntil = 0,
    throttleLow = false,
    live = {
        voltage = nil,
        current = nil,
        rpm = nil,
        escTemperature = nil,
        becTemperature = nil,
        becCurrent = nil,
        becVoltage = nil,
        throttle = nil,
        powerOutput = nil,
        rssi = nil,
        flightMode = nil,
        armed = nil,
        linkLastSeen = nil,
        telemetryLastSeen = nil,
    },
    textgen = {
        active = false,
        confirm = false,
        source = "NONE",
        instance = nil,
        lines = {},
        lastFrame = nil,
        rawOwned = false,
    },
    msp = {
        api = nil,
        client = nil,
        lastRequest = nil,
        lastSeen = nil,
        error = nil,
    },
}

local function loadMspClient()
    if type(loadScript) ~= "function" or type(crossfireTelemetryPush) ~= "function" or
        type(crossfireTelemetryPop) ~= "function" then
        return
    end

    local ok, chunk = pcall(loadScript, "/SCRIPTS/TOOLS/inav-avian-esc/msp.lua")
    if not ok or type(chunk) ~= "function" then
        return
    end

    local loaded, api = pcall(chunk)
    if not loaded or type(api) ~= "table" or type(api.new) ~= "function" then
        return
    end

    local transport = {
        push = function(frameType, payload)
            return crossfireTelemetryPush(frameType, payload)
        end,
        pop = function()
            return crossfireTelemetryPop()
        end,
    }
    local client = api.new(transport)
    if client then
        state.msp.api = api
        state.msp.client = client
    end
end

local function setNotice(message, duration)
    state.notice = message
    state.noticeUntil = now() + (duration or 250)
end

local telemetryFields = {
    voltage = { "EVIN", "EscV", "VFAS", "RxBt" },
    current = { "ECUR", "EscA", "Curr" },
    rpm = { "Erpm", "EscR", "RPM" },
    escTemperature = { "TFET", "EscT", "Tmp1" },
    becTemperature = { "TBEC" },
    becCurrent = { "CBEC", "BecA" },
    becVoltage = { "VBEC", "BecV" },
    throttle = { "ETHR", "Thr" },
    powerOutput = { "EOUT" },
    rssi = { "1RSS", "RSSI", "TRSS" },
    flightMode = { "FM" },
}

local resolvedFields = {}
local nextFieldResolve = 0

local function resolveTelemetryFields(force)
    if type(getFieldInfo) ~= "function" then
        return
    end

    local tick = now()
    if not force and tick < nextFieldResolve then
        return
    end
    nextFieldResolve = tick + 200

    for key, aliases in pairs(telemetryFields) do
        if resolvedFields[key] == nil then
            for index = 1, #aliases do
                local ok, field = pcall(getFieldInfo, aliases[index])
                if ok and field and field.id then
                    resolvedFields[key] = {
                        id = field.id,
                        name = aliases[index],
                    }
                    break
                end
            end
        end
    end
end

local function readResolvedField(key)
    local field = resolvedFields[key]
    if field == nil or type(getValue) ~= "function" then
        return nil
    end

    local ok, value = pcall(getValue, field.id)
    if ok then
        return value
    end
    return nil
end

local function readNamedValue(name)
    if type(getValue) ~= "function" then
        return nil
    end
    local ok, value = pcall(getValue, name)
    if ok then
        return value
    end
    return nil
end

local function rawBufferRead(address)
    if type(multiBuffer) ~= "function" then
        return nil
    end
    local ok, value = pcall(multiBuffer, address)
    if ok then
        return value
    end
    return nil
end

local function rawBufferWrite(address, value)
    if type(multiBuffer) ~= "function" then
        return false
    end
    return pcall(multiBuffer, address, value)
end

local function clearTextGenLines()
    for row = 0, TEXTGEN_ROWS - 1 do
        state.textgen.lines[row] = nil
    end
end

local function loadSimulatedTextGen()
    local lines = {
        [0] = "AVIAN ESC",
        [1] = ">Brake Type",
        [2] = " Normal",
        [3] = "Timing",
        [4] = " 15 Degrees",
        [5] = "BEC Voltage",
        [6] = " 7.4V",
        [7] = "EXIT W/ SAVE",
        [8] = "Use AIL/ELE",
    }
    for row = 0, TEXTGEN_ROWS - 1 do
        state.textgen.lines[row] = lines[row]
    end
end

-- EdgeTX's Spektrum raw bridge uses the "STR" marker, the requested sensor
-- ID at byte 3, and byte 4 as the producer/consumer semaphore. This behavior
-- is documented by EdgeTX spektrum.cpp and exercised by DSMTools TextGen.
local function openTextGen()
    if state.textgen.active then
        return true
    end

    clearTextGenLines()
    state.textgen.lastFrame = nil
    state.textgen.instance = nil

    if state.simulator then
        loadSimulatedTextGen()
        state.textgen.active = true
        state.textgen.source = "SIM"
        state.textgen.lastFrame = now()
        return true
    end

    if type(multiBuffer) ~= "function" then
        setNotice("Raw TextGen is unavailable on this radio/link")
        return false
    end

    local ok = rawBufferWrite(0, string.byte("S"))
    ok = rawBufferWrite(1, string.byte("T")) and ok
    ok = rawBufferWrite(2, string.byte("R")) and ok
    ok = rawBufferWrite(3, TEXTGEN_ID) and ok
    ok = rawBufferWrite(4, 0) and ok

    if not ok then
        setNotice("Unable to open the Spektrum raw buffer")
        return false
    end

    state.textgen.active = true
    state.textgen.source = "4IN1"
    state.textgen.rawOwned = true
    return true
end

local function closeTextGen()
    if state.textgen.rawOwned then
        rawBufferWrite(4, 0)
        rawBufferWrite(3, 0)
        rawBufferWrite(0, 0)
    end
    state.textgen.active = false
    state.textgen.confirm = false
    state.textgen.rawOwned = false
    state.textgen.source = "NONE"
end

local function pollTextGen()
    if not state.textgen.active or state.simulator or not state.textgen.rawOwned then
        return
    end

    if rawBufferRead(0) ~= string.byte("S") or rawBufferRead(3) ~= TEXTGEN_ID then
        rawBufferWrite(0, string.byte("S"))
        rawBufferWrite(1, string.byte("T"))
        rawBufferWrite(2, string.byte("R"))
        rawBufferWrite(3, TEXTGEN_ID)
        rawBufferWrite(4, 0)
        return
    end

    if rawBufferRead(4) ~= TEXTGEN_ID then
        return
    end

    local instance = rawBufferRead(5)
    local lineNumber = rawBufferRead(6)

    if lineNumber == 255 then
        clearTextGenLines()
    elseif lineNumber == 254 then
        setNotice("ESC requested a display refresh", 120)
    elseif lineNumber and lineNumber >= 0 and lineNumber < TEXTGEN_ROWS then
        local characters = {}
        for index = 0, TEXTGEN_COLS - 1 do
            local byte = rawBufferRead(7 + index) or 0
            if byte == 0 then
                break
            elseif byte >= 32 and byte <= 126 then
                characters[#characters + 1] = string.char(byte)
            else
                characters[#characters + 1] = " "
            end
        end
        local line = table.concat(characters):gsub("%s+$", "")
        state.textgen.lines[lineNumber] = line
    end

    state.textgen.instance = instance
    state.textgen.lastFrame = now()
    rawBufferWrite(4, 0)
end

local function updateSimulation()
    local phase = now() / 80
    local live = state.live
    live.voltage = 24.8 + math.sin(phase) * 0.2
    live.current = 18.6 + math.sin(phase * 0.7) * 3.0
    live.rpm = 12480 + rounded(math.sin(phase * 0.5) * 900)
    live.escTemperature = 46.0 + math.sin(phase * 0.2) * 1.5
    live.becTemperature = 39.0
    live.becCurrent = 1.2
    live.becVoltage = 7.4
    live.throttle = 42
    live.powerOutput = 45
    live.rssi = 96
    live.flightMode = "MANUAL"
    live.armed = false
    live.linkLastSeen = now()
    live.telemetryLastSeen = now()
    state.throttleLow = true

end

local function updateMspStatus()
    local mspState = state.msp
    if mspState.client == nil then
        return
    end

    local tick = now()
    local event = mspState.client:process(tick)
    if event and event.kind == "response" and event.command == MSP2_INAV_STATUS then
        state.live.linkLastSeen = tick
        if event.error then
            mspState.error = "MSP status rejected"
            state.live.armed = nil
        else
            local armingFlags = mspState.api.readU32(event.payload, 10)
            if armingFlags then
                state.live.armed = bit32.btest(armingFlags, ARMING_FLAG_ARMED)
                mspState.lastSeen = tick
                mspState.error = nil
            else
                state.live.armed = nil
                mspState.error = "Short MSP status"
            end
        end
    elseif event and event.error then
        mspState.error = event.detail or event.kind
    end


    if mspState.lastSeen and ticksSince(tick, mspState.lastSeen) > TELEMETRY_STALE_TICKS then
        state.live.armed = nil
    end

    local requestDue = mspState.lastRequest == nil or
        ticksSince(tick, mspState.lastRequest) >= MSP_STATUS_POLL_TICKS
    if not mspState.client:isBusy() and requestDue then
        local accepted = mspState.client:request(MSP2_INAV_STATUS, {}, false, tick)
        if accepted then
            mspState.lastRequest = tick
        end
    end
end

local function updateLiveData()
    if state.simulator then
        updateSimulation()
        return
    end

    resolveTelemetryFields(false)
    updateMspStatus()

    local live = state.live
    for key, _ in pairs(telemetryFields) do
        local value = readResolvedField(key)
        if value ~= nil then
            live[key] = value
        end
    end

    if type(live.voltage) == "number" and live.voltage > 0 then
        live.telemetryLastSeen = now()
    end
    if type(live.rssi) == "number" and live.rssi > 0 then
        live.linkLastSeen = now()
    end

    local throttle = readNamedValue("thr")
    state.throttleLow = type(throttle) == "number" and throttle <= -950

    if state.textgen.active and not state.throttleLow then
        closeTextGen()
        setNotice("TextGen closed: throttle stick is not low")
    else
        pollTextGen()
    end
end

local function escTelemetryIsFresh()
    return state.live.telemetryLastSeen ~= nil
        and ticksSince(now(), state.live.telemetryLastSeen) <= TELEMETRY_STALE_TICKS
end

local function linkIsFresh()
    local live = state.live
    local radioFresh = live.linkLastSeen ~= nil
        and ticksSince(now(), live.linkLastSeen) <= TELEMETRY_STALE_TICKS
    local textGenFresh = state.textgen.lastFrame ~= nil
        and ticksSince(now(), state.textgen.lastFrame) <= TELEMETRY_STALE_TICKS
    return radioFresh or escTelemetryIsFresh() or textGenFresh
end

local function formatValue(value, decimals, unit)
    if type(value) ~= "number" then
        return "--"
    end
    local pattern = decimals == 2 and "%.2f" or (decimals == 1 and "%.1f" or "%.0f")
    return string.format(pattern, value) .. (unit or "")
end

local function statusLabel()
    if state.simulator then
        return "SIMULATOR"
    elseif linkIsFresh() then
        return "LIVE"
    end
    return "OFFLINE"
end

local function drawHeader(title)
    lcd.drawRectangle(0, 0, SCREEN_W, HEADER_H)
    lcd.drawText(8, 5, APP_NAME, FLAG_MEDIUM + FLAG_BOLD)
    lcd.drawText(SCREEN_W / 2, 7, title or "", FLAG_SMALL + FLAG_CENTER)
    lcd.drawText(SCREEN_W - 8, 7, statusLabel(), FLAG_SMALL + FLAG_RIGHT + FLAG_BOLD)
end

local function drawFooter(text)
    local y = SCREEN_H - FOOTER_H
    lcd.drawRectangle(0, y, SCREEN_W, FOOTER_H)
    lcd.drawText(8, y + 5, text, FLAG_SMALL)
    lcd.drawText(SCREEN_W - 8, y + 5, "v" .. APP_VERSION, FLAG_SMALL + FLAG_RIGHT)
end

local function drawNotice()
    if state.notice and now() <= state.noticeUntil then
        local y = SCREEN_H - FOOTER_H - 21
        lcd.drawFilledRectangle(5, y, SCREEN_W - 10, 19)
        lcd.drawText(SCREEN_W / 2, y + 2, state.notice, FLAG_SMALL + FLAG_CENTER + FLAG_INVERSE)
    elseif state.notice then
        state.notice = nil
    end
end

local function drawRow(x, y, width, label, value, selected)
    if selected then
        lcd.drawRectangle(x, y - 2, width, 23)
    end
    local flags = selected and FLAG_BOLD or 0
    lcd.drawText(x + 6, y, label, flags)
    lcd.drawText(x + width - 6, y, value or "--", flags + FLAG_RIGHT)
end

local function drawMetric(x, y, width, label, value)
    lcd.drawText(x, y, label, FLAG_SMALL)
    lcd.drawText(x + width, y - 2, value, FLAG_MEDIUM + FLAG_BOLD + FLAG_RIGHT)
end

local function drawHome()
    drawHeader("ESC TELEMETRY / TEXTGEN")
    local y = HEADER_H + 8
    for index = 1, #pageItems do
        drawRow(12, y, SCREEN_W - 24, pageItems[index][1], ">", state.homeFocus == index)
        y = y + 31
    end
    drawFooter("Roller: select   ENTER: open   EXIT: close")
end

local function drawStatus()
    drawHeader(pageTitles[PAGE_STATUS])
    local live = state.live
    drawMetric(18, 47, 190, "Receiver", linkIsFresh() and "CONNECTED" or "WAITING")
    drawMetric(270, 47, 190, "Flight mode", tostring(live.flightMode or "--"))
    drawMetric(18, 89, 190, "Pack voltage", formatValue(live.voltage, 1, " V"))
    drawMetric(270, 89, 190, "Current", formatValue(live.current, 1, " A"))
    drawMetric(18, 131, 190, "ESC speed", formatValue(live.rpm, 0, " RPM"))
    drawMetric(270, 131, 190, "ESC temperature", formatValue(live.escTemperature, 1, " C"))
    drawMetric(18, 173, 190, "Radio link", formatValue(live.rssi, 0, "%"))
    local armed = live.armed == nil and "--" or (live.armed and "ARMED" or "DISARMED")
    drawMetric(270, 173, 190, "Flight controller", armed)
    lcd.drawText(SCREEN_W / 2, 218, "ESC control is read-only in version 0.3", FLAG_SMALL + FLAG_CENTER)
    drawFooter("EXIT: ESC tool")
end

local function drawEsc()
    drawHeader(pageTitles[PAGE_ESC])
    local live = state.live
    drawMetric(20, 48, 190, "Input voltage", formatValue(live.voltage, 2, " V"))
    drawMetric(270, 48, 190, "Motor current", formatValue(live.current, 2, " A"))
    drawMetric(20, 90, 190, "Electrical RPM", formatValue(live.rpm, 0, ""))
    drawMetric(270, 90, 190, "FET temperature", formatValue(live.escTemperature, 1, " C"))
    drawMetric(20, 132, 190, "BEC voltage", formatValue(live.becVoltage, 2, " V"))
    drawMetric(270, 132, 190, "BEC current", formatValue(live.becCurrent, 1, " A"))
    drawMetric(20, 174, 190, "BEC temperature", formatValue(live.becTemperature, 1, " C"))
    drawMetric(270, 174, 190, "ESC output", formatValue(live.powerOutput, 0, "%"))
    local source = resolvedFields.voltage and resolvedFields.voltage.name or (state.simulator and "SIM" or "NONE")
    lcd.drawText(SCREEN_W / 2, 220, "Telemetry source: " .. source, FLAG_SMALL + FLAG_CENTER)
    drawFooter("Read-only telemetry   EXIT: back")
end

local function drawTextGen()
    drawHeader(pageTitles[PAGE_TEXTGEN])
    local textgen = state.textgen

    if not textgen.active then
        lcd.drawText(SCREEN_W / 2, 55, "PROPELLER REMOVAL RECOMMENDED", FLAG_MEDIUM + FLAG_BOLD + FLAG_CENTER)
        lcd.drawText(SCREEN_W / 2, 91, "Throttle stick must be fully low", FLAG_CENTER)
        lcd.drawText(SCREEN_W / 2, 116, "Enable throttle cut and keep the aircraft disarmed", FLAG_CENTER)
        lcd.drawText(SCREEN_W / 2, 137, "Set the thrust-reverse switch to NORMAL", FLAG_SMALL + FLAG_CENTER)
        lcd.drawRectangle(115, 153, SCREEN_W - 230, 42)
        local prompt = textgen.confirm and "PRESS ENTER AGAIN TO OPEN" or "ENTER: CHECK AND OPEN"
        lcd.drawText(SCREEN_W / 2, 166, prompt, FLAG_BOLD + FLAG_CENTER)
        lcd.drawText(
            SCREEN_W / 2,
            206,
            state.throttleLow and "Physical throttle check: LOW" or "Physical throttle check: NOT LOW",
            FLAG_SMALL + FLAG_CENTER + (state.throttleLow and 0 or FLAG_BLINK)
        )
        local armedState = state.live.armed == nil and "UNAVAILABLE"
            or (state.live.armed and "ARMED" or "DISARMED")
        lcd.drawText(
            SCREEN_W / 2,
            222,
            "INAV arming state: " .. armedState,
            FLAG_SMALL + FLAG_CENTER + (state.live.armed and FLAG_BLINK or 0)
        )
        drawFooter("This tool never commands throttle")
        return
    end

    local frameFresh = textgen.lastFrame and
        ticksSince(now(), textgen.lastFrame) <= TELEMETRY_STALE_TICKS
    local source = textgen.source .. (frameFresh and "  RECEIVING" or "  WAITING")
    lcd.drawText(SCREEN_W - 12, 35, source, FLAG_SMALL + FLAG_RIGHT + FLAG_BOLD)

    local y = 38
    for row = 0, TEXTGEN_ROWS - 1 do
        local line = textgen.lines[row] or ""
        local flags = FLAG_MEDIUM
        if row == 0 then
            flags = flags + FLAG_BOLD + FLAG_CENTER
            lcd.drawText(SCREEN_W / 2, y, line, flags)
        else
            lcd.drawText(145, y, line, flags)
        end
        y = y + 21
    end
    lcd.drawText(SCREEN_W - 12, 220, "Use AIL/ELE sticks as prompted by the ESC", FLAG_SMALL + FLAG_RIGHT)
    drawFooter("EXIT: close TextGen   Throttle rise: automatic close")
end

local function drawPage()
    lcd.clear()
    if state.page == PAGE_HOME then
        drawHome()
    elseif state.page == PAGE_STATUS then
        drawStatus()
    elseif state.page == PAGE_ESC then
        drawEsc()
    elseif state.page == PAGE_TEXTGEN then
        drawTextGen()
    end
    drawNotice()
end

local function activateTextGen()
    if state.textgen.active then
        return
    end
    if not state.throttleLow then
        state.textgen.confirm = false
        setNotice("Lower the physical throttle stick first")
        return
    end
    if state.live.armed == true then
        state.textgen.confirm = false
        setNotice("Disarm INAV before opening TextGen")
        return
    end
    if not state.textgen.confirm then
        state.textgen.confirm = true
        setNotice("Confirm throttle cut, disarm, and reverse NORMAL; press ENTER")
        return
    end
    state.textgen.confirm = false
    if openTextGen() then
        setNotice("TextGen display opened")
    end
end

local function activateCurrentPage()
    if state.page == PAGE_HOME then
        state.page = pageItems[state.homeFocus][2]
    elseif state.page == PAGE_TEXTGEN then
        activateTextGen()
    end
end

local function handleNextPrevious(direction)
    if state.page == PAGE_HOME then
        state.homeFocus = ((state.homeFocus - 1 + direction) % #pageItems) + 1
    end
end

local function handleTouch(event, touchState)
    if not eventIs(event, EVT_TOUCH_TAP) or not touchState then
        return false
    end

    local x = touchState.x or 0
    local y = touchState.y or 0
    if state.page ~= PAGE_HOME and y <= HEADER_H and x < 90 then
        if state.textgen.active then
            closeTextGen()
        end
        state.page = PAGE_HOME
        return true
    end

    if state.page == PAGE_HOME and y >= HEADER_H + 8 then
        local index = math.floor((y - HEADER_H - 8) / 31) + 1
        if index >= 1 and index <= #pageItems then
            state.homeFocus = index
            state.page = pageItems[index][2]
            return true
        end
    elseif state.page == PAGE_TEXTGEN and not state.textgen.active and y >= 145 and y <= 205 then
        activateTextGen()
        return true
    end
    return false
end

local function init()
    if type(getVersion) == "function" then
        local ok, _, radio = pcall(getVersion)
        state.simulator = ok and type(radio) == "string" and string.sub(radio, -4) == "simu"
    end
    if not state.simulator then
        loadMspClient()
    end
    resolveTelemetryFields(true)
    updateLiveData()
end

local function run(event, touchState)
    updateLiveData()

    if not handleTouch(event, touchState) then
        if eventIs(event, EVT_VIRTUAL_NEXT) then
            handleNextPrevious(1)
        elseif eventIs(event, EVT_VIRTUAL_PREV) then
            handleNextPrevious(-1)
        elseif eventIs(event, EVT_VIRTUAL_ENTER) then
            activateCurrentPage()
        elseif eventIs(event, EVT_VIRTUAL_EXIT) then
            if state.page == PAGE_HOME then
                closeTextGen()
                return 2
            elseif state.page == PAGE_TEXTGEN and state.textgen.active then
                closeTextGen()
                setNotice("TextGen display closed")
            else
                state.page = PAGE_HOME
            end
        end
    end

    drawPage()
    return 0
end

return { init = init, run = run }
