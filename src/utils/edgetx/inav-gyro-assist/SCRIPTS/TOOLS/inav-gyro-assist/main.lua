-- SPDX-License-Identifier: GPL-3.0-or-later
--
-- INAV Gyro Assist settings tool for EdgeTX 2.11.0 and newer.
-- Uses only generic MSPv2 setting commands over CRSF telemetry.

local APP_NAME = "INAV GYRO ASSIST"
local APP_VERSION = "0.3.0"

local MSP_EEPROM_WRITE = 0x00FA
local MSP2_COMMON_SETTING = 0x1003
local MSP2_COMMON_SET_SETTING = 0x1004
local MSP2_COMMON_SETTING_INFO = 0x1007
local MSP2_INAV_STATUS = 0x2000

local ARMING_FLAG_ARMED = 0x04
local TELEMETRY_STALE_TICKS = 300
local STATUS_POLL_TICKS = 100
local REQUIRED_MAJOR = 2
local REQUIRED_MINOR = 11

local PAGE_HOME = 1
local PAGE_STATUS = 2
local PAGE_SETTINGS = 3

local FLAG_SMALL = SMLSIZE or 0
local FLAG_MEDIUM = MIDSIZE or 0
local FLAG_BOLD = BOLD or 0
local FLAG_RIGHT = RIGHT or 0
local FLAG_CENTER = CENTER or 0
local FLAG_INVERSE = INVERS or 0

local SCREEN_W = LCD_W or 480
local SCREEN_H = LCD_H or 272
local HEADER_H = 30
local FOOTER_H = 25
local VISIBLE_SETTINGS = 6

local definitions = {
    { name = "gyro_assist_roll_gain", label = "Roll gain", unit = "%" },
    { name = "gyro_assist_pitch_gain", label = "Pitch gain", unit = "%" },
    { name = "gyro_assist_yaw_gain", label = "Yaw gain", unit = "%" },
    { name = "gyro_assist_roll_priority", label = "Roll priority", unit = "%" },
    { name = "gyro_assist_pitch_priority", label = "Pitch priority", unit = "%" },
    { name = "gyro_assist_yaw_priority", label = "Yaw priority", unit = "%" },
    { name = "gyro_assist_roll_stop_release_ms", label = "Roll stop release", unit = " ms", step = 10 },
    { name = "gyro_assist_pitch_stop_release_ms", label = "Pitch stop release", unit = " ms", step = 10 },
    { name = "gyro_assist_yaw_stop_release_ms", label = "Yaw stop release", unit = " ms", step = 10 },
    { name = "gyro_assist_roll_stop_lock_ms", label = "Roll stop lock", unit = " ms", step = 10 },
    { name = "gyro_assist_pitch_stop_lock_ms", label = "Pitch stop lock", unit = " ms", step = 10 },
    { name = "gyro_assist_yaw_stop_lock_ms", label = "Yaw stop lock", unit = " ms", step = 10 },
    { name = "gyro_assist_roll_limit", label = "Roll limit", unit = "%" },
    { name = "gyro_assist_pitch_limit", label = "Pitch limit", unit = "%" },
    { name = "gyro_assist_yaw_limit", label = "Yaw limit", unit = "%" },
    { name = "gyro_assist_lpf_hz", label = "Gyro low-pass", unit = " Hz" },
    { name = "gyro_assist_transition_ms", label = "Transition", unit = " ms", step = 10 },
}

local state = {
    page = PAGE_HOME,
    homeFocus = 1,
    focus = 1,
    scroll = 1,
    editing = false,
    dirty = false,
    simulator = false,
    compatible = true,
    notice = nil,
    noticeUntil = 0,
    settings = {},
    saved = {},
    loadedCount = 0,
    live = {
        armed = nil,
        lastSeen = nil,
    },
    msp = {
        api = nil,
        client = nil,
        operation = nil,
        index = 1,
        sent = false,
        lastStatusRequest = nil,
        error = nil,
        partialWrite = false,
    },
}

local function now()
    return type(getTime) == "function" and getTime() or 0
end

local function elapsed(current, previous)
    if current >= previous then
        return current - previous
    end
    return current + 0x80000000 - previous
end

local function clamp(value, minimum, maximum)
    if value < minimum then
        return minimum
    elseif value > maximum then
        return maximum
    end
    return value
end

local function setNotice(message, duration)
    state.notice = message
    state.noticeUntil = now() + (duration or 300)
end

local function versionIsSupported(version)
    if type(version) ~= "string" then
        return false
    end
    local major, minor = string.match(version, "^(%d+)%.(%d+)")
    major = tonumber(major)
    minor = tonumber(minor)
    return major ~= nil and (major > REQUIRED_MAJOR or
        (major == REQUIRED_MAJOR and minor >= REQUIRED_MINOR))
end

local function bytesForName(name)
    local bytes = {}
    for index = 1, #name do
        bytes[#bytes + 1] = string.byte(name, index)
    end
    bytes[#bytes + 1] = 0
    return bytes
end

local function appendValue(payload, value, valueType)
    if valueType == 0 then
        payload[#payload + 1] = value
    elseif valueType == 2 then
        payload[#payload + 1] = value % 256
        payload[#payload + 1] = math.floor(value / 256) % 256
    else
        return false
    end
    return true
end

local function readI32(api, payload, index)
    local value = api.readU32(payload, index)
    if value and value >= 0x80000000 then
        return value - 0x100000000
    end
    return value
end

local function readTypedValue(api, payload, index, valueType)
    if valueType == 0 then
        return payload[index]
    elseif valueType == 2 then
        return api.readU16(payload, index)
    end
    return nil
end

local function parseSettingInfo(definition, payload)
    local terminator = nil
    for index = 1, #payload do
        if payload[index] == 0 then
            terminator = index
            break
        end
    end
    if terminator == nil then
        return nil, "missing setting name"
    end

    local name = ""
    for index = 1, terminator - 1 do
        name = name .. string.char(payload[index])
    end
    if name ~= definition.name then
        return nil, "setting name mismatch"
    end

    local base = terminator + 1
    local api = state.msp.api
    local valueType = payload[base + 2]
    local mode = payload[base + 4]
    local minimum = readI32(api, payload, base + 5)
    local maximum = api.readU32(payload, base + 9)
    local valueIndex = base + 17
    if valueType == nil or mode == nil or minimum == nil or maximum == nil then
        return nil, "short setting metadata"
    end
    if mode ~= 0 or (valueType ~= 0 and valueType ~= 2) then
        return nil, "unsupported setting type"
    end

    local value = readTypedValue(api, payload, valueIndex, valueType)
    if value == nil then
        return nil, "missing setting value"
    end
    return {
        value = value,
        minimum = minimum,
        maximum = maximum,
        valueType = valueType,
    }
end

local function loadMspClient()
    if type(loadScript) ~= "function" or type(crossfireTelemetryPush) ~= "function" or
        type(crossfireTelemetryPop) ~= "function" then
        return false
    end

    local ok, chunk = pcall(loadScript, "/SCRIPTS/TOOLS/inav-gyro-assist/msp.lua")
    if not ok or type(chunk) ~= "function" then
        return false
    end
    local loaded, api = pcall(chunk)
    if not loaded or type(api) ~= "table" or type(api.new) ~= "function" then
        return false
    end

    local client = api.new({
        push = function(frameType, payload)
            return crossfireTelemetryPush(frameType, payload)
        end,
        pop = function()
            return crossfireTelemetryPop()
        end,
    })
    if not client then
        return false
    end
    state.msp.api = api
    state.msp.client = client
    return true
end

local function parseStatus(payload)
    local armingFlags = state.msp.api.readU32(payload, 10)
    if armingFlags == nil then
        return false
    end
    state.live.armed = bit32.btest(armingFlags, ARMING_FLAG_ARMED)
    state.live.lastSeen = now()
    return true
end

local function failOperation(message)
    local msp = state.msp
    msp.error = message
    if msp.partialWrite then
        setNotice("Partial changes NOT saved; reload settings", 500)
    else
        setNotice(message, 400)
    end
    msp.operation = nil
    msp.sent = false
    state.editing = false
end

local function settingLoadComplete()
    local msp = state.msp
    msp.operation = nil
    msp.sent = false
    msp.error = nil
    state.loadedCount = #definitions
    state.dirty = false
    setNotice("Gyro Assist settings loaded")
end

local function handleResponse(event)
    local msp = state.msp
    if event.error then
        failOperation(event.detail or "MSP request rejected")
        return
    end

    if event.command == MSP2_INAV_STATUS then
        if not parseStatus(event.payload) then
            failOperation("Short INAV status response")
            return
        end
        msp.lastStatusRequest = now()
        if msp.operation == "startup_status" then
            msp.operation = "load"
            msp.index = 1
            msp.sent = false
        elseif msp.operation == "precheck" then
            if state.live.armed then
                failOperation("Disarm before applying settings")
            else
                msp.operation = "write"
                msp.index = 1
                msp.sent = false
            end
        elseif msp.operation == "presave" then
            if state.live.armed then
                failOperation("Armed during apply; changes NOT saved")
            else
                msp.operation = "save"
                msp.sent = false
            end
        else
            msp.sent = false
        end
        return
    end

    if msp.operation == "load" and event.command == MSP2_COMMON_SETTING_INFO then
        local definition = definitions[msp.index]
        local parsed, errorMessage = parseSettingInfo(definition, event.payload)
        if not parsed then
            failOperation("Gyro Assist unavailable: " .. errorMessage)
            return
        end
        state.settings[msp.index] = parsed
        state.saved[msp.index] = parsed.value
        state.loadedCount = msp.index
        msp.index = msp.index + 1
        msp.sent = false
        if msp.index > #definitions then
            settingLoadComplete()
        end
        return
    end

    if msp.operation == "write" and event.command == MSP2_COMMON_SET_SETTING then
        msp.partialWrite = true
        msp.index = msp.index + 1
        msp.sent = false
        if msp.index > #definitions then
            msp.operation = "verify"
            msp.index = 1
        end
        return
    end

    if msp.operation == "verify" and event.command == MSP2_COMMON_SETTING then
        local setting = state.settings[msp.index]
        local value = readTypedValue(msp.api, event.payload, 1, setting.valueType)
        if value == nil or value ~= setting.value then
            failOperation("Readback mismatch; changes NOT saved")
            return
        end
        msp.index = msp.index + 1
        msp.sent = false
        if msp.index > #definitions then
            msp.operation = "presave"
        end
        return
    end

    if msp.operation == "save" and event.command == MSP_EEPROM_WRITE then
        for index = 1, #definitions do
            state.saved[index] = state.settings[index].value
        end
        state.dirty = false
        msp.operation = nil
        msp.sent = false
        msp.partialWrite = false
        msp.error = nil
        setNotice("Settings verified and saved")
        return
    end

    failOperation("Unexpected MSP response")
end

local function sendOperationRequest(tick)
    local msp = state.msp
    local operation = msp.operation
    if operation == nil or msp.sent or msp.client:isBusy() then
        return
    end

    local command
    local payload = {}
    local write = false
    if operation == "startup_status" or operation == "precheck" or operation == "presave" then
        command = MSP2_INAV_STATUS
    elseif operation == "load" then
        command = MSP2_COMMON_SETTING_INFO
        payload = bytesForName(definitions[msp.index].name)
    elseif operation == "write" then
        command = MSP2_COMMON_SET_SETTING
        payload = bytesForName(definitions[msp.index].name)
        local setting = state.settings[msp.index]
        if not appendValue(payload, setting.value, setting.valueType) then
            failOperation("Unsupported setting type")
            return
        end
        write = true
    elseif operation == "verify" then
        command = MSP2_COMMON_SETTING
        payload = bytesForName(definitions[msp.index].name)
    elseif operation == "save" then
        command = MSP_EEPROM_WRITE
        write = true
    end

    if command then
        local accepted, detail = msp.client:request(command, payload, write, tick)
        if accepted then
            msp.sent = true
        elseif detail ~= "busy" then
            failOperation(detail or "MSP request failed")
        end
    end
end

local function serviceMsp()
    local msp = state.msp
    if msp.client == nil then
        return
    end
    local tick = now()
    local event = msp.client:process(tick)
    if event then
        handleResponse(event)
    end
    sendOperationRequest(tick)

    local statusDue = msp.lastStatusRequest == nil or
        elapsed(tick, msp.lastStatusRequest) >= STATUS_POLL_TICKS
    if msp.operation == nil and not msp.client:isBusy() and statusDue then
        local accepted = msp.client:request(MSP2_INAV_STATUS, {}, false, tick)
        if accepted then
            msp.lastStatusRequest = tick
        end
    end
    if state.live.lastSeen and elapsed(tick, state.live.lastSeen) > TELEMETRY_STALE_TICKS then
        state.live.armed = nil
    end
end

local function linkIsFresh()
    return state.live.lastSeen ~= nil and elapsed(now(), state.live.lastSeen) <= TELEMETRY_STALE_TICKS
end

local function drawHeader(title)
    lcd.drawRectangle(0, 0, SCREEN_W, HEADER_H)
    lcd.drawText(8, 5, APP_NAME, FLAG_MEDIUM + FLAG_BOLD)
    lcd.drawText(SCREEN_W - 8, 7, title or "", FLAG_SMALL + FLAG_RIGHT)
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

local function drawRow(y, label, value, selected)
    if selected then
        lcd.drawRectangle(12, y - 2, SCREEN_W - 24, 25)
    end
    local flags = selected and FLAG_BOLD or 0
    lcd.drawText(20, y, label, flags)
    lcd.drawText(SCREEN_W - 20, y, value or "--", flags + FLAG_RIGHT)
end

local function drawHome()
    drawHeader("MANUAL MODIFIER")
    drawRow(62, "Status and safety", ">", state.homeFocus == 1)
    drawRow(108, "Gyro Assist settings", ">", state.homeFocus == 2)
    lcd.drawText(SCREEN_W / 2, 175, "Direct stick-to-surface control", FLAG_SMALL + FLAG_CENTER)
    lcd.drawText(SCREEN_W / 2, 194, "+ bounded gyro disturbance correction", FLAG_SMALL + FLAG_CENTER)
    drawFooter("Roller: select   ENTER: open   EXIT: close")
end

local function operationLabel()
    local operation = state.msp.operation
    if operation == "load" or operation == "startup_status" then
        return "LOADING"
    elseif operation ~= nil then
        return "APPLYING"
    elseif state.loadedCount == #definitions then
        return "READY"
    end
    return "UNAVAILABLE"
end

local function drawStatus()
    drawHeader("STATUS / SAFETY")
    local armed = state.live.armed == nil and "--" or (state.live.armed and "ARMED" or "DISARMED")
    drawRow(50, "INAV link", linkIsFresh() and "CONNECTED" or "WAITING", false)
    drawRow(82, "Flight controller", armed, false)
    drawRow(114, "Settings interface", operationLabel(), false)
    drawRow(146, "Loaded settings", state.loadedCount .. " / " .. #definitions, false)
    lcd.drawText(SCREEN_W / 2, 190, "Assign GYRO ASSIST in INAV Modes", FLAG_SMALL + FLAG_CENTER)
    lcd.drawText(SCREEN_W / 2, 210, "Eligible only while fixed-wing MANUAL is active", FLAG_SMALL + FLAG_CENTER)
    drawFooter("Configuration only   EXIT: back")
end

local function settingValue(index)
    local setting = state.settings[index]
    if not setting then
        return "--"
    end
    return tostring(setting.value) .. definitions[index].unit
end

local function drawSettings()
    drawHeader("GENERIC MSP SETTINGS")
    local first = state.scroll
    local last = math.min(#definitions, first + VISIBLE_SETTINGS - 1)
    local y = 38
    for index = first, last do
        drawRow(y, definitions[index].label, settingValue(index), state.focus == index)
        y = y + 29
    end
    if state.focus == #definitions + 1 then
        lcd.drawRectangle(12, 216, 215, 25)
    elseif state.focus == #definitions + 2 then
        lcd.drawRectangle(250, 216, 218, 25)
    end
    lcd.drawText(20, 220, "Apply and save", state.focus == #definitions + 1 and FLAG_BOLD or 0)
    lcd.drawText(458, 220, "Revert", (state.focus == #definitions + 2 and FLAG_BOLD or 0) + FLAG_RIGHT)
    drawFooter(state.editing and "EDIT: roller changes value   ENTER: done" or
        "Roller: select   ENTER: edit/action   EXIT: back")
end

local function drawPage()
    lcd.clear()
    if state.page == PAGE_HOME then
        drawHome()
    elseif state.page == PAGE_STATUS then
        drawStatus()
    else
        drawSettings()
    end
    drawNotice()
end

local function updateScroll()
    if state.focus <= #definitions then
        if state.focus < state.scroll then
            state.scroll = state.focus
        elseif state.focus >= state.scroll + VISIBLE_SETTINGS then
            state.scroll = state.focus - VISIBLE_SETTINGS + 1
        end
    end
end

local function adjustFocusedSetting(direction)
    local setting = state.settings[state.focus]
    if not setting then
        return
    end
    local step = definitions[state.focus].step or 1
    setting.value = clamp(setting.value + direction * step, setting.minimum, setting.maximum)
    state.dirty = false
    for index = 1, #definitions do
        if state.settings[index] and state.settings[index].value ~= state.saved[index] then
            state.dirty = true
            break
        end
    end
end

local function beginApply()
    local msp = state.msp
    if state.simulator then
        for index = 1, #definitions do
            state.saved[index] = state.settings[index].value
        end
        state.dirty = false
        setNotice("Simulator values saved for this session")
        return
    end
    if state.loadedCount ~= #definitions or msp.operation ~= nil then
        setNotice("Settings are not ready")
        return
    end
    if state.live.armed ~= false then
        setNotice("Disarm and restore the INAV link first")
        return
    end
    if not state.dirty then
        setNotice("No changes to apply")
        return
    end
    msp.operation = "precheck"
    msp.index = 1
    msp.sent = false
    msp.partialWrite = false
    setNotice("Checking disarmed state...")
end

local function revertSettings()
    if state.loadedCount ~= #definitions then
        return
    end
    for index = 1, #definitions do
        state.settings[index].value = state.saved[index]
    end
    state.dirty = false
    state.editing = false
    setNotice("Local edits reverted")
end

local function activate()
    if state.page == PAGE_HOME then
        state.page = state.homeFocus == 1 and PAGE_STATUS or PAGE_SETTINGS
        return
    end
    if state.page ~= PAGE_SETTINGS or state.msp.operation ~= nil then
        return
    end
    if state.focus <= #definitions then
        if state.settings[state.focus] then
            state.editing = not state.editing
        end
    elseif state.focus == #definitions + 1 then
        beginApply()
    else
        revertSettings()
    end
end

local function move(direction)
    if state.page == PAGE_HOME then
        state.homeFocus = ((state.homeFocus - 1 + direction) % 2) + 1
    elseif state.page == PAGE_SETTINGS then
        if state.editing then
            adjustFocusedSetting(direction)
        else
            state.focus = ((state.focus - 1 + direction) % (#definitions + 2)) + 1
            updateScroll()
        end
    end
end

local function initSimulator()
    for index = 1, #definitions do
        local defaults = { 0, 0, 0, 20, 20, 20, 20, 250 }
        local maxima = { 100, 100, 100, 50, 50, 50, 100, 2000 }
        state.settings[index] = {
            value = defaults[index],
            minimum = index == 8 and 20 or 0,
            maximum = maxima[index],
            valueType = index == 8 and 2 or 0,
        }
        state.saved[index] = defaults[index]
    end
    state.loadedCount = #definitions
    state.live.armed = false
    state.live.lastSeen = now()
end

local function init()
    local version, radio
    if type(getVersion) == "function" then
        local ok
        ok, version, radio = pcall(getVersion)
        if not ok then
            version = nil
        end
    end
    state.compatible = versionIsSupported(version)
    state.simulator = type(radio) == "string" and string.sub(radio, -4) == "simu"

    if not state.compatible then
        setNotice("EdgeTX 2.11.0 or newer is required", 1000)
        return
    end
    if state.simulator then
        initSimulator()
    elseif loadMspClient() then
        state.msp.operation = "startup_status"
    else
        state.msp.error = "CRSF MSP transport unavailable"
        setNotice(state.msp.error, 600)
    end
    serviceMsp()
end

local function eventIs(event, expected)
    return expected ~= nil and event == expected
end

local function run(event)
    serviceMsp()
    if eventIs(event, EVT_VIRTUAL_NEXT) then
        move(1)
    elseif eventIs(event, EVT_VIRTUAL_PREV) then
        move(-1)
    elseif eventIs(event, EVT_VIRTUAL_ENTER) then
        activate()
    elseif eventIs(event, EVT_VIRTUAL_EXIT) then
        if state.page == PAGE_HOME then
            return 2
        end
        state.editing = false
        state.page = PAGE_HOME
    end
    drawPage()
    return 0
end

local function background()
    serviceMsp()
end

return { init = init, run = run, background = background }
