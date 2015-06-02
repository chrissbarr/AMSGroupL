  var pracTitle = 'Brinkleworth\'s Wild Ride'
  var pracBlurb = 'The ability to move in a straight line is something most of us take for granted, but have you ever wondered how hard it is for a robot to drive straight? There is a lot more to it than you might first think. This practical will guide you through configuring a basic motor controller that will allow your robot to travel in the direction it is pointing and, hopefully, not too far off course. You will be introduced to many new concepts like what happens when you do not use feedback from sensors (open-loop) and what can be achieved if you do (closed loop). You will also have the opportunity to measure the performance of your control over the robot by quantifying things like how fast the robot responds to your commands (settling time) and how accurately (steady-state error and overshoot).'
  
  var pracSteps = [
    '<h4>Step 1</h4>\
    Without touching any of controller settings, use the inbuilt remote to drive the robot forward and back.<br><br>\
    Notice how the robot drives in a straight line during these tests.<br><br>\
    On the step response graph, notice how both motors are going at roughly the same speed. This is because the robots controller in using motor feedback to ensure the desired speed is maintained.',
    '<h4>Step 2</h4>\
    Using the button in the Control section of the interface, turn off the motor control.<br><br>\
    Without the controller your robot will be driving using an open loop, meaning no motor feedback is used, rather the speed values you are selecting are going directly to the motors.<br><br>\
    Once again, drive the robot forward and back.<br><br>\
    Notice the robot no longer drives straight, pay attention to the step response graph that show the speed of each wheel. Why is the robot not driving straight?',
    '<h4>Step 3</h4>\
    With the motor controller turned off the motors use open loop control. Meaning the motors can be given a signal but nothing is done to monitor or correct the speed that the signal produces in the motors. This is mainly a problem because every motor is subtly different and therefore acts in a slightly different way. It is almost impossible, to get a pair of exactly matching motors in any system.<br><br>\
    With the motor controller enabled the motor\'s speeds are monitored and modified so they both run at the desired speed. This is done using encoders, devices attached to the motors, measuring the rate they turn, or RPS (revolutions per second).<br><br>\
    The robot uses a <a href="/static/img/PID_diagram.jpg" target="_blank">PI motor controller</a>, which is configured using two values, representing the Proportional (P) and Integral (I) responses respectively. The "P" value is larger when the difference between the desired and current speeds (error) is greater. In an operating control system the “P” component mostly determines the amplitude of the initial jump when the motors are first turned on, or change direction. The "I" value changes over time if the error is not 0 (i.e. the desired and current motor speeds are not the same) and it controls the amplitude of the smaller corrections made after a change in desired speed and throughout.',
    '<h4>Step 4</h4>\
    Turn the motor controller back on. <br><br>\
    Apply the values P=0.5 and I=0 to the controller and drive the robot forwards, observing the step response graph. Take a note of the difference between the desired and final speed of the robot (error), as well as the time it took to reach it’s final speed (settling time) and if it went higher than the desired speed (overshoot).',
    '<h4>Step 5</h4>\
    Apply the values P=0.1, I=0 to the controller and drive the robot forwards, observing the step response graph and the parameters of interest.',
    '<h4>Step 6</h4>\
    Apply the values P=4, I=0 to the controller and drive the robot forwards, observing the step response graph and the parameters of interest.',
    '<h4>Step 7</h4>\
    Apply the values P=0, I=0.1 to the controller and drive the robot forwards, observing the step response graph and the parameters of interest.',
    '<h4>Step 8</h4>\
    Apply the values P=0, I=1.5 to the controller and drive the robot forwards, observing the step response graph and the parameters of interest.',
    '<h4>Step 9</h4>\
    Apply the values P=0.5, I=0.1 to the controller and drive the robot forwards, observing the step response graph and the parameters of interest.',
    '<h4>Step 10 - Final Step</h4>\
    Using what you\'ve learnt try to find the optimal P and I values for your robot. That is the values that result in the smallest settling time and error with a minimal overshoot and error.<br><br>\
    To do this, it\'s recommended to you start with both values as 0, and then increase your P value until the system just overshoots the desired speed and starts to oscillate (vary around the desired speed) and the reduce your P value. Then repeat the process for your I value.<br><br>\
    What were your optimal controller values and what system performance were you able to achieve?.'
  ];
  
  
  // Insert Title and blurb
  document.getElementById('pracTitle').innerHTML = pracTitle;
  document.getElementById('pracBlurb').innerHTML = pracBlurb;
  
  // Handle the step by step instructions
  var msgIndex = 0;
  function changeStep(direction) 
  {
    msgIndex = msgIndex + direction;
    if (msgIndex < 0)
    {
      msgIndex++;
    }
    if (msgIndex > pracSteps.length-1)
    {
      msgIndex--;
    }
    document.getElementById('stepBox').innerHTML = pracSteps[msgIndex];
  }
  window.onload = function () {
    changeStep(0);
  }