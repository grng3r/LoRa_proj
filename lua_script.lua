function parsePayload(appeui,deveui,payload)
	lat = "Latitude"
  	lon = "Longitude"
  	moist = "Moisture"
  	light = "Light"
  	temp = "Temperature"
  	hum = "Humidity"
  	r = "R"
  	g = "G"
  	b = "B"
  
	--value = resiot_hexdecode_ascii(payload)
  	lat_v = resiot_ba2float32LE(resiot_baslice(payload, 0, 4))
  	lon_v = resiot_ba2float32LE(resiot_baslice(payload, 4, 8))
  	light_v = resiot_ba2float32LE(resiot_baslice(payload, 8, 12))
  	moist_v = resiot_ba2float32LE(resiot_baslice(payload, 12, 16))
  	temp_v = resiot_baslice(payload, 16, 20)
  	temp_v_A = {temp_v[4],temp_v[3], temp_v[2], temp_v[1]}
  	temp_v_n = resiot_ba2intBE32(temp_v_A)
  	hum_v = resiot_baslice(payload, 20, 24)
    hum_v_A = {hum_v[4], hum_v[3], hum_v[2], hum_v[1]}
  	hum_v_n = resiot_ba2intBE32(hum_v_A)
  	r_v = resiot_ba2intLE16(resiot_baslice(payload, 24, 25))
  	g_v = resiot_ba2intLE16(resiot_baslice(payload, 25, 26))
  	b_v = resiot_ba2intLE16(resiot_baslice(payload, 26, 27))
  	
	--Call for LUA Script engine prints
	resiot_debug(string.format("%s: %f\n%s: %f\n%s: %f\n%s: %f\n%s: %d\n%s: %d\n", lat, lat_v, lon, lon_v, light, light_v, moist, moist_v, temp, temp_v, hum, hum_v)) --, r, r_v, g, g_v, b, b_v))
	worked, err = resiot_setnodevalue(appeui, deveui, lat, lat_v)
	if (not worked) then
		resiot_debug(string.format("Set Value Error %s \n",err))
	else
		resiot_debug("Set Node value successfull\n")
	end
end
Origin = resiot_startfrom() --Scene process starts here
--Manual script execution for testing
if Origin == "Manual" then
  	--d4da2142e5f26f4012e613e00000500203558086c9f18448618
	payload = {0xd4, 0xda, 0x21, 0x42, 0xe5, 0xf2, 0x6f, 0x40, 0x12, 0xe6, 0x13, 0xe0, 0x00, 0x00, 0x50, 0x02, 0x03, 0x55, 0x80, 0x86, 0xc9, 0xf1, 0x84, 0x48, 0x61, 0x08}
	--payload = "d4da2142e5f26f4012e613e00000500203558086c9f18448618" --Set your test payload here in hexadecimal
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
