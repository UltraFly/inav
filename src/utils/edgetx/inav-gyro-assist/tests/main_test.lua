local mainPath = assert(arg[1], "main.lua path is required")
local mspPath = assert(arg[2], "msp.lua path is required")

LCD_W = 480
LCD_H = 272
SMLSIZE = 1
MIDSIZE = 2
BOLD = 4
RIGHT = 8
CENTER = 16
INVERS = 32

EVT_VIRTUAL_NEXT = 101
EVT_VIRTUAL_PREV = 102
EVT_VIRTUAL_ENTER = 103
EVT_VIRTUAL_EXIT = 104

local tick = 0
local pushed = {}
local incoming = {}
local drawnText = {}

function getTime()
    return tick
end

function getVersion()
    return "2.11.0", "tx16s"
end

function loadScript(path)
    if path == "/SCRIPTS/TOOLS/inav-gyro-assist/msp.lua" then
        return assert(loadfile(mspPath))
    end
    return nil
end

function crossfireTelemetryPush(frameType, payload)
    pushed[#pushed + 1] = { frameType = frameType, payload = payload }
    return true
end

function crossfireTelemetryPop()
    if #incoming == 0 then
        return nil
    end
    local frame = table.remove(incoming, 1)
    return frame.frameType, frame.payload
end

lcd = {}
function lcd.clear()
    drawnText = {}
end
function lcd.drawText(_, _, value)
    drawnText[#drawnText + 1] = tostring(value)
end
function lcd.drawRectangle() end
function lcd.drawFilledRectangle() end

local function containsText(expected)
    for _, value in ipairs(drawnText) do
        if string.find(value, expected, 1, true) then
            return true
        end
    end
    return false
end

local app = assert(loadfile(mainPath))()
assert(type(app.init) == "function")
assert(type(app.run) == "function")
assert(type(app.background) == "function")

local pushIndex = 1

local function nextRequest()
    local request = nil
    for _ = 1, 200 do
        tick = tick + 1
        app.run(0)
        while pushIndex <= #pushed do
            local frame = pushed[pushIndex]
            pushIndex = pushIndex + 1
            local data = frame.payload
            local status = data[3]
            if bit32.btest(status, 0x10) then
                request = {
                    frameType = frame.frameType,
                    command = data[5] + data[6] * 256,
                    size = data[7] + data[8] * 256,
                    payload = {},
                }
            elseif request then
                for index = 4, #data do
                    request.payload[#request.payload + 1] = data[index]
                end
            end
            if request and #request.payload >= request.size then
                return request
            end
        end
    end
    error("timed out waiting for MSP request")
end

local function appendU16(bytes, value)
    bytes[#bytes + 1] = value % 256
    bytes[#bytes + 1] = math.floor(value / 256) % 256
end

local function appendU32(bytes, value)
    appendU16(bytes, value % 65536)
    appendU16(bytes, math.floor(value / 65536))
end

local function respond(command, payload, isError)
    payload = payload or {}
    local status = 0x50 + (isError and 0x80 or 0)
    local data = { 0xEA, 0xC8, status, 0 }
    appendU16(data, command)
    appendU16(data, #payload)
    for index = 1, #payload do
        data[#data + 1] = payload[index]
    end
    incoming[#incoming + 1] = { frameType = 0x7B, payload = data }
end

local function statusPayload(armed)
    local payload = {
        0xE8, 0x03,
        0, 0,
        1, 0,
        20, 0,
        0,
    }
    appendU32(payload, armed and 0x04 or 0)
    return payload
end

local definitions = {
    { "gyro_assist_roll_gain", 0, 0, 100, 0 },
    { "gyro_assist_pitch_gain", 0, 0, 100, 0 },
    { "gyro_assist_yaw_gain", 0, 0, 100, 0 },
    { "gyro_assist_roll_priority", 0, 0, 200, 100 },
    { "gyro_assist_pitch_priority", 0, 0, 200, 100 },
    { "gyro_assist_yaw_priority", 0, 0, 200, 100 },
    { "gyro_assist_roll_stop_release_ms", 2, 0, 2000, 50 },
    { "gyro_assist_pitch_stop_release_ms", 2, 0, 2000, 50 },
    { "gyro_assist_yaw_stop_release_ms", 2, 0, 2000, 50 },
    { "gyro_assist_roll_stop_lock_ms", 2, 0, 2000, 250 },
    { "gyro_assist_pitch_stop_lock_ms", 2, 0, 2000, 250 },
    { "gyro_assist_yaw_stop_lock_ms", 2, 0, 2000, 250 },
    { "gyro_assist_roll_limit", 0, 0, 50, 20 },
    { "gyro_assist_pitch_limit", 0, 0, 50, 20 },
    { "gyro_assist_yaw_limit", 0, 0, 50, 20 },
    { "gyro_assist_lpf_hz", 0, 0, 100, 20 },
    { "gyro_assist_transition_ms", 2, 20, 2000, 250 },
}

local function namePayload(name)
    local payload = {}
    for index = 1, #name do
        payload[#payload + 1] = string.byte(name, index)
    end
    payload[#payload + 1] = 0
    return payload
end

local function settingInfoPayload(definition, index)
    local payload = namePayload(definition[1])
    appendU16(payload, 1048)
    payload[#payload + 1] = definition[2]
    payload[#payload + 1] = 0
    payload[#payload + 1] = 0
    appendU32(payload, definition[3])
    appendU32(payload, definition[4])
    appendU16(payload, index)
    payload[#payload + 1] = 0
    payload[#payload + 1] = 0
    if definition[2] == 2 then
        appendU16(payload, definition[5])
    else
        payload[#payload + 1] = definition[5]
    end
    return payload
end

local function requestName(request)
    local value = ""
    for index = 1, #request.payload do
        if request.payload[index] == 0 then
            break
        end
        value = value .. string.char(request.payload[index])
    end
    return value
end

app.init()

local request = nextRequest()
assert(request.command == 0x2000, "startup did not request INAV status")
assert(request.frameType == 0x7A, "status did not use an MSP request frame")
respond(request.command, statusPayload(false))

for index, definition in ipairs(definitions) do
    request = nextRequest()
    assert(request.command == 0x1007, "setting discovery did not use SETTING_INFO")
    assert(requestName(request) == definition[1], "unexpected setting discovery order")
    respond(request.command, settingInfoPayload(definition, index - 1))
end

tick = tick + 1
app.run(0)
app.run(EVT_VIRTUAL_ENTER)
assert(containsText("DISARMED"), "disarmed state was not rendered")
assert(containsText("17 / 17"), "all generic settings were not loaded")
assert(not containsText("Avian"), "Gyro Assist tool unexpectedly references Avian")

app.run(EVT_VIRTUAL_EXIT)
app.run(EVT_VIRTUAL_NEXT)
app.run(EVT_VIRTUAL_ENTER)
app.run(EVT_VIRTUAL_ENTER)
app.run(EVT_VIRTUAL_NEXT)
app.run(EVT_VIRTUAL_ENTER)

for _ = 1, 17 do
    app.run(EVT_VIRTUAL_NEXT)
end
app.run(EVT_VIRTUAL_ENTER)

request = nextRequest()
assert(request.command == 0x2000, "apply did not recheck armed state")
respond(request.command, statusPayload(false))

local expected = { 1, 0, 0, 100, 100, 100, 50, 50, 50, 250, 250, 250, 20, 20, 20, 20, 250 }
for index, definition in ipairs(definitions) do
    request = nextRequest()
    assert(request.command == 0x1004, "apply did not use generic SET_SETTING")
    assert(request.frameType == 0x7C, "setting write did not use MSP write framing")
    assert(requestName(request) == definition[1], "unexpected setting write order")
    respond(request.command, {})
end

for index, definition in ipairs(definitions) do
    request = nextRequest()
    assert(request.command == 0x1003, "apply did not read settings back")
    assert(requestName(request) == definition[1], "unexpected readback order")
    local payload = {}
    if definition[2] == 2 then
        appendU16(payload, expected[index])
    else
        payload[1] = expected[index]
    end
    respond(request.command, payload)
end

request = nextRequest()
assert(request.command == 0x2000, "apply did not recheck armed state before save")
respond(request.command, statusPayload(false))

request = nextRequest()
assert(request.command == 0x00FA, "verified settings were not persisted")
assert(request.frameType == 0x7C, "EEPROM write did not use MSP write framing")
respond(request.command, {})
tick = tick + 1
app.run(0)
assert(containsText("Settings verified and saved"), "successful save was not reported")

print("PASS INAV Gyro Assist generic MSP settings workflow")
