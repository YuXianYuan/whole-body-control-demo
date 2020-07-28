% 位置优先级高于方向优先级
%========================================= 宏，以及全局变量
TIME_STEP = 10;
systime = 0.0;   %秒计时器
%========================================= 参数区
r1_kp = 150;
r2_kp = 100;
%========================================= device 初始化
m0   = wb_robot_get_device('m0 linear motor');
m1   = wb_robot_get_device('m1 rotational motor');
m2   = wb_robot_get_device('m2 rotational motor');
m3   = wb_robot_get_device('m3 rotational motor');

pos0 = wb_robot_get_device('m0 position sensor');
pos1 = wb_robot_get_device('m1 position sensor');
pos2 = wb_robot_get_device('m2 position sensor');
pos3 = wb_robot_get_device('m3 position sensor');

wb_position_sensor_enable(pos0, TIME_STEP);
wb_position_sensor_enable(pos1, TIME_STEP);
wb_position_sensor_enable(pos2, TIME_STEP);
wb_position_sensor_enable(pos3, TIME_STEP);

%========================================= main loop:
while wb_robot_step(TIME_STEP) ~= -1

  % read the sensors
  yh     = wb_position_sensor_get_value(pos0);
  theta1 = wb_position_sensor_get_value(pos1);
  theta2 = wb_position_sensor_get_value(pos2);
  theta3 = wb_position_sensor_get_value(pos3);
  %Process here sensor data
  c1   = cos(theta1);
  c12  = cos(theta1+theta2);
  c123 = cos(theta1+theta2+theta3);
  s1   = sin(theta1);
  s12  = sin(theta1+theta2);
  s123 = sin(theta1+theta2+theta3);
  % systime
  systime = systime + 0.001*TIME_STEP;
  % controller
  r1d       = [2 -yh]';
  r2d       = [0];
  r1        = [c1+c12+c123 s1+s12+s123]';
  r2        = [theta1+theta2+theta3];
  r1_dot    = r1_kp*(r1d-r1);
  r2_dot    = r2_kp*(r2d-r2);
  J1        = [-s1-s12-s123 -s12-s123 -s123; c1+c12+c123 c12+c123 c123];
  J2        = [1 1 1];
  pinvJ1    = pinv(J1);
  pinvJ2    = pinv(J2);
  I         = eye(3);
  
  theta_dot = pinvJ2*r2_dot + (I - pinvJ2*J2)*pinvJ1*r1_dot;

  %trick 让m2远离奇异点
  kp = 0.3;
  theta_dot(2) = theta_dot(2) + kp/theta2;
  
  wb_motor_set_position(m1, inf);
  wb_motor_set_position(m2, inf);
  wb_motor_set_position(m3, inf);
  wb_motor_set_velocity(m1,theta_dot(1));
  wb_motor_set_velocity(m2,theta_dot(2));
  wb_motor_set_velocity(m3,theta_dot(3));
  %send actuator commands
  wb_motor_set_position(m0, 2.0*sin(0.1*systime*2*pi));
  % if your code plots some graphics, it needs to flushed like this:
  drawnow;
end
