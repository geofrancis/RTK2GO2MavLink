# RTCM2MavLink

this has been modified to take RTCM from RTK2GO or other online sources and send it over mavlink to the flight controller using a ESP32 C3, my rover has a 4g modem so this eliminates the need for mission planner to send corrections. 

requires https://github.com/GLAY-AK2/NTRIP-client-for-Arduino     
and https://github.com/mavlink/c_library_v2      

output is pin 10.
