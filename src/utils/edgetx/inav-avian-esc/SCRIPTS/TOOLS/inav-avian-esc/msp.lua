-- SPDX-License-Identifier: GPL-3.0-or-later
--
-- Minimal native MSPv2-over-CRSF client for the INAV Avian ESC EdgeTX tool.
-- The transport object passed to new() must provide push(frameType, bytes)
-- and pop(), matching EdgeTX crossfireTelemetryPush/Pop semantics.

local CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
local CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA

local CRSF_FRAMETYPE_MSP_REQ = 0x7A
local CRSF_FRAMETYPE_MSP_RESP = 0x7B
local CRSF_FRAMETYPE_MSP_WRITE = 0x7C

local MSP_VERSION_V2 = 0x40
local MSP_START = 0x10
local MSP_ERROR = 0x80
local MSP_SEQUENCE_MASK = 0x0F
local MSP_VERSION_MASK = 0x60

local TX_CHUNK_SIZE = 6
local MAX_REQUEST_PAYLOAD = 60
local MAX_RESPONSE_PAYLOAD = 512
local DEFAULT_TIMEOUT_TICKS = 100 -- EdgeTX getTime() ticks are 10 ms.
local MAX_POP_PER_PROCESS = 16

local module = {}

local function byte(value)
    if type(value) ~= "number" or value < 0 or value > 255 or value ~= math.floor(value) then
        return nil
    end
    return value
end

local function copyBytes(values)
    local result = {}
    for index = 1, #values do
        local value = byte(values[index])
        if value == nil then
            return nil
        end
        result[index] = value
    end
    return result
end

local function elapsed(nowTicks, thenTicks)
    if nowTicks >= thenTicks then
        return nowTicks - thenTicks
    end
    return nowTicks + 0x80000000 - thenTicks
end

local function makeEvent(kind, command, payload, isError, detail)
    return {
        kind = kind,
        command = command,
        payload = payload or {},
        error = isError or false,
        detail = detail,
    }
end

local Client = {}
Client.__index = Client

function Client:isBusy()
    return self.pendingCommand ~= nil
end

function Client:cancel()
    self.txChunks = {}
    self.txIndex = 1
    self.pendingCommand = nil
    self.pendingSince = nil
    self.rxStarted = false
    self.rxPayload = {}
end

function Client:request(command, payload, write, nowTicks)
    if self:isBusy() then
        return false, "busy"
    end
    if type(command) ~= "number" or command < 0 or command > 0xFFFF or
        command ~= math.floor(command) then
        return false, "invalid command"
    end

    payload = payload or {}
    if type(payload) ~= "table" or #payload > MAX_REQUEST_PAYLOAD then
        return false, "invalid payload"
    end
    local requestPayload = copyBytes(payload)
    if requestPayload == nil then
        return false, "invalid payload"
    end

    local sequence = self.nextSequence
    local first = {
        MSP_VERSION_V2 + MSP_START + sequence,
        0, -- MSPv2 flags
        command % 256,
        math.floor(command / 256),
        #requestPayload % 256,
        math.floor(#requestPayload / 256),
    }
    self.txChunks = { first }
    sequence = (sequence + 1) % 16

    local payloadIndex = 1
    while payloadIndex <= #requestPayload do
        local chunk = { MSP_VERSION_V2 + sequence }
        sequence = (sequence + 1) % 16
        while #chunk < TX_CHUNK_SIZE and payloadIndex <= #requestPayload do
            chunk[#chunk + 1] = requestPayload[payloadIndex]
            payloadIndex = payloadIndex + 1
        end
        self.txChunks[#self.txChunks + 1] = chunk
    end

    self.nextSequence = sequence
    self.txIndex = 1
    self.txFrameType = write and CRSF_FRAMETYPE_MSP_WRITE or CRSF_FRAMETYPE_MSP_REQ
    self.pendingCommand = command
    self.pendingSince = nowTicks or 0
    self.rxStarted = false
    self.rxPayload = {}
    return true
end

function Client:sendNextChunk()
    local chunk = self.txChunks[self.txIndex]
    if chunk == nil then
        return false
    end

    local frame = {
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        CRSF_ADDRESS_RADIO_TRANSMITTER,
    }
    for index = 1, #chunk do
        frame[#frame + 1] = chunk[index]
    end

    if not self.transport.push(self.txFrameType, frame) then
        return false
    end

    self.txIndex = self.txIndex + 1
    return true
end

function Client:finishResponse(isError)
    local event = makeEvent("response", self.rxCommand, self.rxPayload, isError)
    self:cancel()
    return event
end

function Client:fail(detail)
    local command = self.pendingCommand
    self:cancel()
    return makeEvent("protocol_error", command, {}, true, detail)
end

function Client:consumeResponse(data)
    if type(data) ~= "table" or data[1] ~= CRSF_ADDRESS_RADIO_TRANSMITTER or
        data[2] ~= CRSF_ADDRESS_FLIGHT_CONTROLLER or #data < 3 then
        return nil
    end

    local status = byte(data[3])
    if status == nil or bit32.band(status, MSP_VERSION_MASK) ~= MSP_VERSION_V2 then
        return self:fail("unsupported MSP version")
    end

    local sequence = bit32.band(status, MSP_SEQUENCE_MASK)
    local start = bit32.btest(status, MSP_START)
    local isError = bit32.btest(status, MSP_ERROR)
    local payloadIndex

    if start then
        if #data < 8 then
            return self:fail("short MSPv2 response header")
        end

        local flags = byte(data[4])
        local commandLow = byte(data[5])
        local commandHigh = byte(data[6])
        local sizeLow = byte(data[7])
        local sizeHigh = byte(data[8])
        if flags == nil or commandLow == nil or commandHigh == nil or sizeLow == nil or sizeHigh == nil then
            return self:fail("invalid MSPv2 response header")
        end

        self.rxFlags = flags
        self.rxCommand = commandLow + commandHigh * 256
        self.rxSize = sizeLow + sizeHigh * 256
        self.rxPayload = {}
        self.rxSequence = sequence
        self.rxError = isError
        self.rxStarted = true
        payloadIndex = 9

        if self.rxCommand ~= self.pendingCommand then
            return self:fail("response command mismatch")
        end
        if self.rxSize > MAX_RESPONSE_PAYLOAD then
            return self:fail("response payload too large")
        end
    else
        if not self.rxStarted then
            return self:fail("continuation without start")
        end
        if sequence ~= (self.rxSequence + 1) % 16 then
            return self:fail("response sequence gap")
        end
        self.rxSequence = sequence
        self.rxError = self.rxError or isError
        payloadIndex = 4
    end

    while payloadIndex <= #data and #self.rxPayload < self.rxSize do
        local value = byte(data[payloadIndex])
        if value == nil then
            return self:fail("invalid response byte")
        end
        self.rxPayload[#self.rxPayload + 1] = value
        payloadIndex = payloadIndex + 1
    end

    if #self.rxPayload == self.rxSize then
        return self:finishResponse(self.rxError)
    end
    return nil
end

function Client:process(nowTicks)
    if not self:isBusy() then
        return nil
    end

    if elapsed(nowTicks or 0, self.pendingSince) > self.timeoutTicks then
        local command = self.pendingCommand
        self:cancel()
        return makeEvent("timeout", command, {}, true, "response timeout")
    end

    -- Do not accept a response until the complete request has been queued.
    -- This prevents a delayed response for an older request with the same
    -- command ID from being consumed before the new request is transmitted.
    if self.txChunks[self.txIndex] ~= nil then
        self:sendNextChunk()
        return nil
    end

    for _ = 1, MAX_POP_PER_PROCESS do
        local frameType, data = self.transport.pop()
        if frameType == nil then
            break
        end
        if frameType == CRSF_FRAMETYPE_MSP_RESP then
            local event = self:consumeResponse(data)
            if event ~= nil then
                return event
            end
        end
    end

    return nil
end

function module.new(transport, timeoutTicks)
    if type(transport) ~= "table" or type(transport.push) ~= "function" or
        type(transport.pop) ~= "function" then
        return nil, "invalid transport"
    end

    local client = setmetatable({
        transport = transport,
        timeoutTicks = timeoutTicks or DEFAULT_TIMEOUT_TICKS,
        nextSequence = 0,
        txChunks = {},
        txIndex = 1,
        rxPayload = {},
        rxStarted = false,
    }, Client)
    return client
end

function module.readU16(payload, index)
    if type(payload) ~= "table" or byte(payload[index]) == nil or byte(payload[index + 1]) == nil then
        return nil
    end
    return payload[index] + payload[index + 1] * 256
end

function module.readU32(payload, index)
    local low = module.readU16(payload, index)
    local high = module.readU16(payload, index + 2)
    if low == nil or high == nil then
        return nil
    end
    return low + high * 65536
end

module.CRSF_FRAMETYPE_MSP_REQ = CRSF_FRAMETYPE_MSP_REQ
module.CRSF_FRAMETYPE_MSP_RESP = CRSF_FRAMETYPE_MSP_RESP
module.CRSF_FRAMETYPE_MSP_WRITE = CRSF_FRAMETYPE_MSP_WRITE
module.CRSF_ADDRESS_FLIGHT_CONTROLLER = CRSF_ADDRESS_FLIGHT_CONTROLLER
module.CRSF_ADDRESS_RADIO_TRANSMITTER = CRSF_ADDRESS_RADIO_TRANSMITTER

return module
