local scriptPath = assert(arg[1], "msp.lua path is required")
local msp = assert(loadfile(scriptPath))()

local passed = 0
local failed = 0

local function fail(message)
    error(message, 2)
end

local function assertEqual(expected, actual, message)
    if expected ~= actual then
        fail((message or "values differ") .. ": expected " .. tostring(expected) .. ", got " .. tostring(actual))
    end
end

local function assertTableEqual(expected, actual, message)
    assertEqual(#expected, #actual, (message or "tables differ") .. " length")
    for index = 1, #expected do
        assertEqual(expected[index], actual[index], (message or "tables differ") .. " byte " .. index)
    end
end

local function test(name, callback)
    local ok, message = pcall(callback)
    if ok then
        passed = passed + 1
        print("PASS " .. name)
    else
        failed = failed + 1
        print("FAIL " .. name .. ": " .. tostring(message))
    end
end

local function newTransport()
    local transport = {
        pushed = {},
        incoming = {},
        acceptPush = true,
    }

    function transport.push(frameType, payload)
        if not transport.acceptPush then
            return false
        end
        transport.pushed[#transport.pushed + 1] = {
            frameType = frameType,
            payload = payload,
        }
        return true
    end

    function transport.pop()
        if #transport.incoming == 0 then
            return nil
        end
        local frame = table.remove(transport.incoming, 1)
        return frame.frameType, frame.payload
    end

    return transport
end

local function response(status, command, payload, size)
    local bytes = {
        msp.CRSF_ADDRESS_RADIO_TRANSMITTER,
        msp.CRSF_ADDRESS_FLIGHT_CONTROLLER,
        status,
        0,
        command % 256,
        math.floor(command / 256),
        (size or #payload) % 256,
        math.floor((size or #payload) / 256),
    }
    for index = 1, #payload do
        bytes[#bytes + 1] = payload[index]
    end
    return {
        frameType = msp.CRSF_FRAMETYPE_MSP_RESP,
        payload = bytes,
    }
end

test("encodes native MSPv2 read header", function()
    local transport = newTransport()
    local client = assert(msp.new(transport))
    assertEqual(true, client:request(0x2000, {}, false, 10))
    client:process(10)

    assertEqual(1, #transport.pushed)
    assertEqual(msp.CRSF_FRAMETYPE_MSP_REQ, transport.pushed[1].frameType)
    assertTableEqual({ 0xC8, 0xEA, 0x50, 0, 0x00, 0x20, 0, 0 }, transport.pushed[1].payload)
end)

test("fragments writes into bounded CRSF chunks", function()
    local transport = newTransport()
    local client = assert(msp.new(transport))
    assertEqual(true, client:request(0x2345, { 1, 2, 3, 4, 5, 6, 7 }, true, 20))
    client:process(20)
    client:process(21)
    client:process(22)

    assertEqual(3, #transport.pushed)
    assertEqual(msp.CRSF_FRAMETYPE_MSP_WRITE, transport.pushed[1].frameType)
    assertTableEqual({ 0xC8, 0xEA, 0x50, 0, 0x45, 0x23, 7, 0 }, transport.pushed[1].payload)
    assertTableEqual({ 0xC8, 0xEA, 0x41, 1, 2, 3, 4, 5 }, transport.pushed[2].payload)
    assertTableEqual({ 0xC8, 0xEA, 0x42, 6, 7 }, transport.pushed[3].payload)
end)

test("honors EdgeTX output backpressure", function()
    local transport = newTransport()
    transport.acceptPush = false
    local client = assert(msp.new(transport))
    assertEqual(true, client:request(0x2000, {}, false, 0))
    client:process(1)
    assertEqual(0, #transport.pushed)

    transport.acceptPush = true
    client:process(2)
    assertEqual(1, #transport.pushed)
end)

test("reassembles a single response", function()
    local transport = newTransport()
    local client = assert(msp.new(transport))
    client:request(0x2000, {}, false, 0)
    client:process(0)
    transport.incoming[1] = response(0x53, 0x2000, { 10, 20, 30 })

    local event = client:process(1)
    assertEqual("response", event.kind)
    assertEqual(0x2000, event.command)
    assertEqual(false, event.error)
    assertTableEqual({ 10, 20, 30 }, event.payload)
    assertEqual(false, client:isBusy())
end)

test("reassembles a multi-frame response", function()
    local transport = newTransport()
    local client = assert(msp.new(transport))
    client:request(0x2240, {}, false, 0)
    client:process(0)
    transport.incoming[1] = response(0x5E, 0x2240, { 1, 2 }, 5)
    transport.incoming[2] = {
        frameType = msp.CRSF_FRAMETYPE_MSP_RESP,
        payload = { 0xEA, 0xC8, 0x4F, 3, 4, 5 },
    }

    local event = client:process(1)
    assertEqual("response", event.kind)
    assertTableEqual({ 1, 2, 3, 4, 5 }, event.payload)
end)

test("sequence comparison wraps at sixteen", function()
    local transport = newTransport()
    local client = assert(msp.new(transport))
    client:request(0x2240, {}, false, 0)
    client:process(0)
    transport.incoming[1] = response(0x5F, 0x2240, { 1 }, 2)
    transport.incoming[2] = {
        frameType = msp.CRSF_FRAMETYPE_MSP_RESP,
        payload = { 0xEA, 0xC8, 0x40, 2 },
    }

    local event = client:process(1)
    assertEqual("response", event.kind)
    assertTableEqual({ 1, 2 }, event.payload)
end)

test("ignores unrelated CRSF telemetry", function()
    local transport = newTransport()
    local client = assert(msp.new(transport))
    client:request(0x2000, {}, false, 0)
    client:process(0)
    transport.incoming[1] = { frameType = 0x08, payload = { 1, 2, 3 } }
    transport.incoming[2] = response(0x50, 0x2000, { 9 })

    local event = client:process(1)
    assertEqual("response", event.kind)
    assertTableEqual({ 9 }, event.payload)
end)

test("rejects a response sequence gap", function()
    local transport = newTransport()
    local client = assert(msp.new(transport))
    client:request(0x2000, {}, false, 0)
    client:process(0)
    transport.incoming[1] = response(0x51, 0x2000, { 1 }, 2)
    transport.incoming[2] = {
        frameType = msp.CRSF_FRAMETYPE_MSP_RESP,
        payload = { 0xEA, 0xC8, 0x43, 2 },
    }

    local event = client:process(1)
    assertEqual("protocol_error", event.kind)
    assertEqual("response sequence gap", event.detail)
end)

test("rejects a mismatched response command", function()
    local transport = newTransport()
    local client = assert(msp.new(transport))
    client:request(0x2000, {}, false, 0)
    client:process(0)
    transport.incoming[1] = response(0x50, 0x2001, {})

    local event = client:process(1)
    assertEqual("protocol_error", event.kind)
    assertEqual("response command mismatch", event.detail)
end)

test("rejects malformed response header bytes", function()
    local transport = newTransport()
    local client = assert(msp.new(transport))
    client:request(0x2000, {}, false, 0)
    client:process(0)
    transport.incoming[1] = {
        frameType = msp.CRSF_FRAMETYPE_MSP_RESP,
        payload = { 0xEA, 0xC8, 0x50, 0, "bad", 0x20, 0, 0 },
    }

    local event = client:process(1)
    assertEqual("protocol_error", event.kind)
    assertEqual("invalid MSPv2 response header", event.detail)
end)

test("reports MSP error responses", function()
    local transport = newTransport()
    local client = assert(msp.new(transport))
    client:request(0x2000, {}, false, 0)
    client:process(0)
    transport.incoming[1] = response(0xD0, 0x2000, { 2 })

    local event = client:process(1)
    assertEqual("response", event.kind)
    assertEqual(true, event.error)
    assertTableEqual({ 2 }, event.payload)
end)

test("times out and handles timer wrap", function()
    local transport = newTransport()
    local client = assert(msp.new(transport, 10))
    client:request(0x2000, {}, false, 0x7FFFFFFC)
    assertEqual(nil, client:process(3))
    local event = client:process(7)
    assertEqual("timeout", event.kind)
    assertEqual(false, client:isBusy())
end)

test("rejects busy invalid and oversized requests", function()
    local transport = newTransport()
    local client = assert(msp.new(transport))
    assertEqual(false, client:request(0x10000, {}, false, 0))
    local oversized = {}
    for index = 1, 61 do
        oversized[index] = index % 256
    end
    assertEqual(false, client:request(0x2000, oversized, false, 0))
    assertEqual(true, client:request(0x2000, {}, false, 0))
    assertEqual(false, client:request(0x2001, {}, false, 0))
end)

test("decodes little-endian values", function()
    assertEqual(0x1234, msp.readU16({ 0x34, 0x12 }, 1))
    assertEqual(0x89ABCDEF, msp.readU32({ 0xEF, 0xCD, 0xAB, 0x89 }, 1))
    assertEqual(nil, msp.readU32({ 1, 2, 3 }, 1))
end)

print(string.format("%d passed, %d failed", passed, failed))
if failed > 0 then
    os.exit(1)
end
