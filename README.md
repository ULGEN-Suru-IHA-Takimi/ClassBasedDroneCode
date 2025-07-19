# ClassBasedDroneCode

## Code yapısı:
-xbee package class
-xbee controller class
-xbee drone controller (sbee controller child`ı)


## Örnek komutlar
arm
takeoff
goto_coord 47.398316 8.546406 10.0 0


### Görev 0
run 0 variable=Hello_world
### Görev 2
addwp center_point 47.397606 8.543060 0 0
run 2 d=1 center_wp_id=center_point radius_m=20 num_points=16 altitude_m=15
### Görev 3
run 3 d=1 start_wp_id=start_point side_length_m=100 altitude_m=25 num_laps=2