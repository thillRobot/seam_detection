
 ### troubleshoot with auborobot
 
 To use this package with the aubo robot, uncomment and enable compiling of the aubo robot system node in `CMakeLists.txt`. These nodes will not compile by default to allo    w for use in docker and on other platforms.
 
 
 #### issues
 i see the following error when i try to publish to andriod_gui/gcode_cmd
 ```
 [ERROR] [1692989638.343471973]: Client [/rostopic_17739_1692988617268] wants topic /android_gui/gcode_cmd to have datatype/md5sum [aubo_control/gcodeAction/8261e41e538034    94ec669905817b139c], but our version has [aubo_control/gcodeAction/a83a0e1a726f23e73947f0f4e478e627]. Dropping connection.
 ```
#### issues
 i see the following error when i try to publish to andriod_gui/gcode_cmd
 ```
 [ERROR] [1692989638.343471973]: Client [/rostopic_17739_1692988617268] wants topic /android_gui/gcode_cmd to have datatype/md5sum [aubo_control/gcodeAction/8261e41e538034    94ec669905817b139c], but our version has [aubo_control/gcodeAction/a83a0e1a726f23e73947f0f4e478e627]. Dropping connection.
 ```
 
 i think this is a noetic-kinetic version mismatch but I am not sure. The custom message compiles and published on the local machine fine, neither remote machine can see t    he msg from the other computer
 
 the published message (noetic side) looks just fine
 
 ```
 ---
 header:
   seq: 304
   stamp:
     secs: 0
     nsecs:         0
   frame_id: ''
 file_name: "scan_target"
 job_cycles: 0
 start_job: True
 loop: False
 start_atline: 0
 ---
 ```
 
 we might have to make a patch on the kinetic side to fix this.



