diff -r wheelchair_firmware_previous/include/wheelchair_firmware/wheelchair_interface.hpp wheelchair_firmware/include/wheelchair_firmware/wheelchair_interface.hpp
79a80
>   double ticks_per_revolution_;
diff -r wheelchair_firmware_previous/src/wheelchair_interface.cpp wheelchair_firmware/src/wheelchair_interface.cpp
70a71,73
>     
>     ticks_per_revolution_ = info_.hardware_parameters.count("ticks_per_revolution") ?
>                             std::stod(info_.hardware_parameters.at("ticks_per_revolution")) : 326.0;
465a469,472
>       // The values from the Arduino are already in rad/s
>       double right_rad_s = right_vel;
>       double left_rad_s = left_vel;
> 
467c474
<         wheel_velocity_states_[1] = right_vel;  // rightwheel -> index 1
---
>         wheel_velocity_states_[1] = right_rad_s;  // rightwheel -> index 1
470c477
<         wheel_velocity_states_[0] = left_vel;   // leftwheel -> index 0
---
>         wheel_velocity_states_[0] = left_rad_s;   // leftwheel -> index 0
480c487
<           wheel_position_states_[1] += right_vel * dt;  // rightwheel
---
>           wheel_position_states_[1] += right_rad_s * dt;  // rightwheel
483c490
<           wheel_position_states_[0] += left_vel * dt;   // leftwheel
---
>           wheel_position_states_[0] += left_rad_s * dt;   // leftwheel
532,533c539,541
<   double left_cmd = wheel_velocity_commands_[0];   // leftwheel
<   double right_cmd = wheel_velocity_commands_[1];  // rightwheel
---
>   // Negate the commands to compensate for backwards-wired motors
>   double left_cmd = -wheel_velocity_commands_[0];   // leftwheel
>   double right_cmd = -wheel_velocity_commands_[1];  // rightwheel
