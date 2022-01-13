function getAny(param, func, size)
    local retVal = func(resiot_baslice(param.bytes, param.position, param.position + size))
    param.position = param.position + size
    return retVal
end


function setValue(appeui, deveui, name, value)
    worked, err = resiot_setnodevalue(appeui, deveui, name, value)

    if (not worked) then
        resiot_debug(string.format("Set Value Error %s \n", err))
    else
        resiot_debug(string.format("Tag %s set to %s \n", name, value))
    end
end

function getInt32(param)
    return getAny(param, resiot_ba2sintBE32, 4)
end

function getUInt32(param)
    return getAny(param, resiot_ba2intBE32, 4)
end

function getFloat(param)
    return getAny(param, resiot_ba2float32BE, 4)
end

function getChar(param)
    return getAny(param, resiot_ba2str, 1)
end


function parsePayload(appeui,deveui,payload)
	lat = "Latitude"
  	lon = "Longitude"
  	moist = "Moisture"
  	light = "Light"
  	temp = "Temperature"
  	hum = "Humidity"
  	rgb = "RGB"
    acc_x = "ACC_x"
  
	values = resiot_hexdecode(payload)
    p = {bytes=bytes, position=0}

  	setValue(appeui,deveui, lat, getFloat(p))
  	setValue(appeui, deveui, lon, getFloat(p))
  	setValue(appeui, deveui, light, getFloat(p))
    setValue(appeui, deveui, moist, getFloat(p))
    setValue(appeui, deveui, temp , getInt32(p))
    setValue(appeui, deveui, hum , getUInt32(p))
  	color_char = getChar(p)
    setValue(appeui, deveui, moist, getFloat(p))

    if colotChar == "R" then
        setValue(appeui, deveui, "Color", "Red")
    elseif colotChar == "G" then
        setValue(appeui, deveui, "Color", "Green")
    elseif colotChar == "B" then
        setValue(appeui, deveui, "Color", "Blue")
    elseif colotChar == "C" then
        setValue(appeui, deveui, "Color", "Clear")
    end
	
end
Origin = resiot_startfrom() --Scene process starts here
--Manual script execution for testing
if Origin == "Manual" then
	payload = "ceaa21420f0b6d400000000000000000f2440000cb9b000066259561" --Set your test payload here in hexadecimal
	appeui = "f31c2e8bc671281d5116f08ff0b7928f" --Set your Application EUI here
	deveui = "7139323559379194" --Set your own Device EUI here
-- Normal execution, get payload received from device
else
	appeui = resiot_comm_getparam("appeui")
	deveui = resiot_comm_getparam("deveui")
	payload, err = resiot_getlastpayload(appeui, deveui)
end
--Do your stuff
parsePayload(appeui,deveui,payload)
