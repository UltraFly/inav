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
BLINK = 64
SOLID = 0

EVT_VIRTUAL_NEXT = 101
EVT_VIRTUAL_PREV = 102
EVT_VIRTUAL_ENTER = 103
EVT_VIRTUAL_EXIT = 104
EVT_TOUCH_TAP = 105

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

function getFieldInfo()
    return nil
end

function getValue(name)
    if name == "thr" then
        return -1024
    end
    return nil
end

function getOutputValue()
    return 0
end

model = {}
function model.getOutput(index)
    return { name = "OUT" .. tostring(index + 1) }
end

function loadScript(path)
    if path == "/SCRIPTS/TOOLS/inav-avian-esc/msp.lua" then
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
function lcd.drawLine() end

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

app.init()
app.run(0)
assert(#pushed == 1, "status request was not sent")
assert(pushed[1].frameType == 0x7A, "status request did not use CRSF MSP request frame")
local expectedRequest = { 0xC8, 0xEA, 0x50, 0, 0, 0x20, 0, 0 }
for index = 1, #expectedRequest do
    assert(pushed[1].payload[index] == expectedRequest[index], "unexpected status request byte " .. index)
end

local statusPayload = {
    0xE8, 0x03, -- cycle time
    0, 0,       -- I2C errors
    1, 0,       -- sensors
    20, 0,      -- CPU load
    0,          -- profile / battery profile
    4, 0, 0, 0, -- ARMED flag
}
local response = { 0xEA, 0xC8, 0x50, 0, 0, 0x20, #statusPayload, 0 }
for index = 1, #statusPayload do
    response[#response + 1] = statusPayload[index]
end
incoming[1] = { frameType = 0x7B, payload = response }

tick = 1
app.run(0)
app.run(EVT_VIRTUAL_ENTER)
assert(containsText("ARMED"), "MSP armed state was not rendered")
app.run(EVT_VIRTUAL_EXIT)

for _ = 1, 2 do
    app.run(EVT_VIRTUAL_NEXT)
end
app.run(EVT_VIRTUAL_ENTER)
assert(containsText("thrust-reverse switch to NORMAL"), "TextGen reverse-switch prompt was not rendered")
app.run(EVT_VIRTUAL_ENTER)
assert(containsText("Disarm INAV before opening TextGen"), "armed TextGen entry was not blocked")

print("PASS INAV Avian ESC MSP status and TextGen safety smoke test")
