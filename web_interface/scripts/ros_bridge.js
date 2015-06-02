/*
  Initiate Connection to ROS
*/
var ros = new ROSLIB.Ros({
  url : 'ws://' + window.location.hostname + ':9090'
});

ros.on('connection', function(){
  console.log('Connected to websocket server.');
});

ros.on('error', function(error){
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function(){
  console.log('Connection to websocket server closed.');
});

ros.getTopics(function(topics){
  //console.log(topics)
  dumpTopics(topics);
});


/*
  Publish a Topic
*/
function sendRosMessage(topic, dataType, data)
{
  // Define Topic
  var rTopic = new ROSLIB.Topic({
    ros: ros, 
    name : topic,
    messageType : dataType
  });

  // Prepare Variable
  var rData = new ROSLIB.Message({
    data : data
  });

  // Commit
  rTopic.publish(rData);
};

/*
  Subscribe to a Topic
*/
function subRosTopic(topic, dataType, storeObj)
{
  // Declare Listener
  var rListener = new ROSLIB.Topic({
    ros : ros,
    name : topic,
    messageType : dataType
  });
  // Subscribe
  rListener.subscribe(function(message) {
    //console.log('Received message on ' + rListener.name + ': ' + message.data);
    storeObj.Value = message.data
    //rListener.unsubscribe(); //-- Auto unsunbs when page closes/reloads
  });
};

/*
  Declare variables for known topics
*/
var hardwareVersion = 3;          // No longer need, should only be ver3 now
var rosEncRateL = 0;
var rosEncRateR = 0;
var rosEncCountL = 0;
var rosEncCountR = 0;
var rosMotSpdL = 0;
var rosMotSpdR = 0;
var rosMotDirL = 1;
var rosMotDirR = 1;
var rosPoseHeading = 0;
var rosPose1 = 0;
var rosPose2 = 0;
var rosBatteryLevel = 100;
var rosSnrIR0 = 0;
var rosSnrIR1 = 0;
var rosSnrIR2 = 0;
var rosSnrIR3 = 0;
var rosSnrUS0 = 0;
var rosSnrUS1 = 0;
var rosLCDOutput = "";
var rosSnrIMU = new Array();
var graphBufferL = new Array();
var graphBufferR = new Array();


// Subscriber Objects -- One per topic
// Converts ros topics into generic js objects

var objIMU = {Value: 0};
var objIR = {Value: 0};
var objLCD = {Value: 0};
var objUS = {Value: 0};
var objBat = {Value: 0};
var objEnc = {Value: 0};
var objForce = {Value: 0};
var objMotor = {Value: 0};
var objTime = {Value: 0};
var objVicon = {Value: 0};
var objPose = {Value: 0};
var objBuffL = {Value: 0};
var objBuffR = {Value: 0};

function sub2Ros()
{
  subRosTopic('/'+hostname+'/get/IMU_status', 'std_msgs/Float32MultiArray', objIMU);
  subRosTopic('/'+hostname+'/get/IR_status', 'std_msgs/Float32MultiArray', objIR);
  subRosTopic('/'+hostname+'/get/LCD_status', 'std_msgs/String', objLCD);
  subRosTopic('/'+hostname+'/get/US_status', 'std_msgs/Float32MultiArray', objUS);
  subRosTopic('/'+hostname+'/get/battery_status', 'std_msgs/Float32MultiArray', objBat);
  subRosTopic('/'+hostname+'/get/encoder_status', 'std_msgs/Int32MultiArray', objEnc);
  subRosTopic('/'+hostname+'/get/force_status', 'std_msgs/Float32MultiArray', objForce);
  subRosTopic('/'+hostname+'/get/motor_status', 'std_msgs/Int32MultiArray', objMotor);
  subRosTopic('/'+hostname+'/get/time_status', 'std_msgs/Int32MultiArray', objTime);
  subRosTopic('/'+hostname+'/get/vicon_pose', 'std_msgs/String', objVicon);
  //subRosTopic('/'+hostname+'/get/XXXX_pose', 'std_msgs/Float32MultiArray', objPose);
  subRosTopic('/'+hostname+'/get/motor_buffer_left', 'std_msgs/Int32MultiArray', objBuffL);
  subRosTopic('/'+hostname+'/get/motor_buffer_right', 'std_msgs/Int32MultiArray', objBuffR);
}

// Define whitelist
whitelist = ['IMU_status', 'IR_status', 'LCD_status', 'US_status',
              'encoder_status', 'force_status', 'motor_status',
              'vicon_pose', 'camera_stream', 'rosout', 'motor_buffer',
              'rosout_agg', 'diagnostics', 'joy', '/set/'];

var rosEncRateL = 0;
var rosEncRateR = 0;
var rosEncCountL = 0;
var rosEncCountR = 0;
var rosMotSpdL = 0;
var rosMotSpdR = 0;
var rosMotDirL = 1;
var rosMotDirR = 1;
var rosPoseHeading = 0;
var rosPose1 = 0;
var rosPose2 = 0;
var rosBatteryLevel = 100;
var rosSnrIR0 = 0;
var rosSnrIR1 = 0;
var rosSnrIR2 = 0;
var rosSnrIR3 = 0;
var rosSnrUS0 = 0;
var rosSnrUS1 = 0;
var rosLCDOutput = "";
var rosSnrIMU = new Array();
var graphBufferL = new Array();
var graphBufferR = new Array();

// Js Variables
// Converts generic objects into the desired variables
function updateRosVars()
{
  var tmpData
  // Battery
  tmpData = (objBat.Value)//.split(",").map(function (x) {return parseFloat(x, 10);});
  rosBatteryLevel = tmpData[0];
  rosBatteryCurrent = tmpData[1];

  // Motors
  tmpData = (objMotor.Value)//.split(",").map(function (x) {return parseInt(x, 10);});
  switch (tmpData[0])
  {
     case 0:
       rosMotDirL = 1;
       rosMotDirR = 1;
       break;
     case 1:
       rosMotDirL = -1;
       rosMotDirR = -1;
       break;
    case 2:
       rosMotDirL = 1;
       rosMotDirR = -1;
       break;
    case 3:
       rosMotDirL = -1;
       rosMotDirR = 1;
       break;
  }
  
  rosMotSpdL = tmpData[1];
  rosMotSpdR = tmpData[2];

  // Encoders
  tmpData = (objEnc.Value)//.split(",").map(function (x) {return parseInt(x, 10);});
  rosEncRateL = tmpData[0];
  rosEncRateR = tmpData[1];
  rosEncCountL = tmpData[2];
  rosEncCountR = tmpData[3];

  // IR Sensor
  tmpData = (objIR.Value)//.split(",").map(function (x) {return parseFloat(x, 10);});
  rosSnrIR0 = tmpData[0];
  rosSnrIR1 = tmpData[1];
  rosSnrIR2 = tmpData[2];
  rosSnrIR3 = tmpData[3];
  
  // IMU Sensor
  rosSnrIMU = (objIMU.Value)//.split(",").map(function (x) {return parseFloat(x, 10);});
  
  // US Sensor
  tmpData = (objUS.Value)//.split(",").map(function (x) {return parseFloat(x, 10);});
  rosSnrUS0 = tmpData[0];
  rosSnrUS1 = tmpData[1];

  // Pose
  //tmpData = objPose.Value
  //rosPose1 = tmpData[0]
  //rosPose2 = tmpData[1]
  //rosPoseHeading = tmpData[2]

  //LCD
  rosLCDOutput = objLCD.Value;
  
  // Motor Buffers
  graphBufferL = (objBuffL.Value)
  graphBufferR = (objBuffR.Value)

}

var dumpedTopics = 0;
var dumpNames = new Array();
var dumpObjs = new Array();

function dumpTopics(topics)
{

  for (i = 0; i < topics.length; i++)
  { // For each found topic
    
    whitelisted = false;
    for (j = 0; j < whitelist.length; j++)
    { // For each whitelisted topic
      if (topics[i].indexOf(whitelist[j]) > -1)
      { // They match
        whitelisted = true;
      }
    }

    if (!whitelisted)
    {
      // If not whitelisted, create in dumpzone
      tName = topics[i].substring(topics[i].lastIndexOf("/")+1);
      console.log('Unregistered topic '+tName+' discovered. Added to drop zone');
      tName_div = '<div id="' + tName + '" class="hardware_info_title"><b>'+tName+'</b></div>';
      tData_div = '<div id="' + tName + '_data" class="hardware_info_data">n/a</div><br>';
      $('#dumpzone').append(tName_div, tData_div);
      
      // Start subscriber
      // setTimeout to avoid them tripping over each other
      setTimeout(function(indicies)
      {
        getTopicTypeN( topics[indicies[0]], indicies[1], function subDump(name, di, type)
        {
          dumpNames[di] = name.substring(name.lastIndexOf("/")+1);
          dumpObjs[di] = {Value: 0};
          //console.log(dumpNames[di] + ' | ' + type)
          subRosTopic(name, type, dumpObjs[di]);
        })
      }, dumpedTopics*500, [i, dumpedTopics])
      dumpedTopics = dumpedTopics + 1;

    }

  }
}


// Custom RoslibJS function
// Callback returns the topic name & type
function getTopicTypeN(topic, index, callback)
{
  var topicTypeClient = new ROSLIB.Service({
    ros : ros,
    name : '/rosapi/topic_type',
    serviceType : 'rosapi/TopicType'
  });
  var request = new ROSLIB.ServiceRequest({
    topic: topic
  });
  topicTypeClient.callService(request, function(result) {
    callback(topic, index, result.type);
  });
};


















//---Unused API functions-----------------------------
/*
  Calling a service
*/
//var addTwoIntsClient = new ROSLIB.Service({
//  ros : ros,
//  name : '/add_two_ints',
//  serviceType : 'rospy_tutorials/AddTwoInts'
//});

//var request = new ROSLIB.ServiceRequest({
//  a : 1,
//  b : 2
//});

//addTwoIntsClient.callService(request, function(result) {
//  console.log('Result for service call on '
//    + addTwoIntsClient.name
//    + ': '
//    + result.sum);
//});

/*
  Getting and setting a param value
*/
//ros.getParams(function(params) {
//  console.log(params);
//});

//var maxVelX = new ROSLIB.Param({
//  ros : ros,
//  name : 'max_vel_y'
//});

//maxVelX.set(0.8);
//maxVelX.get(function(value) {
//  console.log('MAX VAL: ' + value);
//});

