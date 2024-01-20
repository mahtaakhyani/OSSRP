// SETTING STATIC GLOBAL VARIABLES
// ---------------------------------
var host;
var android_host;
var port = '8000';
var android_port = '8080';
var rosbridge_port = '9090';
var face_url_id = '';
var sound_url_val = '';
var auto_imit_val = false;
// - - - Django Server - - - 
var request_server_ip = '/reqip'; //URL has been set in 'interface_backend/core/urls.py'
var get_latest_emotion_url = '/reqcli'; //URL has been set in 'interface_backend/core/urls.py'
var get_msg_type_url = '/get_msg_type'; //URL has been set in 'interface_backend/core/urls.py'
var django_base_url;
var request_current_exp;
var publish_new_exp;
var android_server_url;  
// Topics
var publish_exp_topic = '/web_exp_publisher';
var publish_motion_topic = '/web_motion_publisher';
var camera_img_topic = '/camera/image_raw';
var listen_exp_topic = '/py_exp_publisher';
var listen_motion_topic = '/cmd_vel_listener';
var dyna_topic = '/cmd_vel/dyna';
var listen_dyna_status_topic = '/dyna_status';
// Messages
var exp_msg_type = 'infrastructure/Exp';
var motion_msg_type = 'geometry_msgs/Twist';
var camera_img_msg_type = 'sensor_msgs/Image';
var dyna_msg_type = 'infrastructure/DynaTwist'
var dyna_status_msg_type = 'infrastructure/DynaStatus';
var tts_msg_type = 'infrastructure/TTS';
// Services
var tts_service = 'text_to_speech';
var tts_srv_type = 'infrastructure/Tts';

// - - - ROS - - -
var robot_ws;


// SETTING DYNAMIC GLOBAL VARIABLES
// ---------------------------------
// - - - Django Server - - - 
$.ajax({
  type: "GET",
  url: request_server_ip,
  success: function(response) {
    host = response.host
    android_host = response.android_ip;
    alert("Android is at: "+ android_host + "Set the host ip on the Android to: " + host)
    setTimeout(() => {
      // Code to execute after the delay
      console.log("After delay");
    }, 2000); // Delay of 2000 milliseconds (2 seconds)
    
    set_variables();
    set_ros();
    set_default_exp(); // Setting the default emotion to 'neutral'. 
                      // The function is defined in the emotion handling section 
                      // and also takes in the default emotion's name as an argument 
                      // (i.e. face_name_val='neutral' or whatever the default emotion must be)

  }
});
function set_variables() {
      console.log('setting environment variables...');
      django_base_url = 'http://' + host + ':' + port ;
      request_current_exp =  '/reqemo';  //URL has been set in 'interface_backendapp/urls.py'
      publish_new_exp =  '/reqpub'; //URL has been set in 'interface_backendapp/urls.py'
      android_server_url = 'http://' + android_host + ':' + android_port + '/android_server';
      console.log('Android Server is listening on: '+android_server_url+
      '\nAsking the server for latest emotion, then sending status, both on: /reqcli');
      
      // - - - ROS - - -
      // Workspace
      robot_ws = 'ws://'+host+':'+ rosbridge_port;	// Setting the websocket url for the ROS environment
      console.log('ROSBridge websocket is listening on: '+robot_ws+
      '\n\nActiveTopics:\n'+publish_exp_topic+' to publish selected emotion on\n '
      +listen_exp_topic+' to listen for the recognized emotion from the robot (i.e. Auto mode)'+
      '\n/head_cmd_vel to publish motion commands on');
      
      // - - - Camera - - -
      var camera_img_url = 'http://' + host + ':8080/stream?topic=/image_raw';
      document.getElementById("camera_img").src = camera_img_url;
      console.log('Camera is streaming on: '+camera_img_url);
      // - - - Camera Landmarked - - -
      var camera_landmarked_img_url = 'http://' + host + ':8080/stream?topic=/image_raw/landmarked';
      document.getElementById("camera_landmarked_img").src = camera_landmarked_img_url;
      console.log('Camera Landmarked is streaming on: '+camera_landmarked_img_url);
      // - - - Camera Gaze Frame - - -
      var camera_gaze_img_url = 'http://' + host + ':8080/stream?topic=/image_raw/gaze_frame';
      document.getElementById("camera_gaze_img").src = camera_gaze_img_url;
      console.log('Camera Gaze Frame is streaming on: '+camera_gaze_img_url);
}
      

  
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ---------------------------------------------- END OF VARIABLE DECLARATION -----------------------------------------------
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// // <----------------------------------------- ROS CONNECTION ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
var ros;
function set_ros() {
  const ROSLIB = require('roslib');
  // Connecting to ROS via 'rosbridge_websocket_server' Launch Node running on the master "URI/IP/URL :Port 9090(default)"
  // ----------------- 
  ros = new ROSLIB.Ros({
    url : robot_ws
  });

  ros.on('connection', function() {
    console.log('Connected to websocket server.');
  });

  ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
  });

  ros.on('close', function() {
    console.log('Connection to websocket server closed.');
  });

}
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // <----------------------------------------- END OF ROS CONNECTION ----------------------------------------->
// // // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
      
// }                 

// ---------------------------------------------- START OF INTERFACE FUNCTIONS -----------------------------------------------
// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Log Side Menu
// -----------------
var $logmenu = document.querySelector('.logmenu');
console.stdlog = console.log.bind(console);
console.logs = [];
console.log = function(){
    console.logs.push(Array.from(arguments));
    console.stdlog.apply(console, arguments);
}
$logmenu.addEventListener('click', function(e) {
  var item = document.createElement('li');
  item.setAttribute('id','item');
  $('#log').add(item);
  $('#log').toggle('display');
  document.getElementById("log").innerHTML = console.logs.slice(-3) + '\n'+console.logs.slice(-2);
} );


//Changing tabs
// -----------------
function viewdiv(div) {
  $('.change').children().hide();
  $(document.getElementById(div)).show().children().show();
}

// Changing the image viewer's image
// -----------------
function changeimage(topic_name) {
  var img_url = 'http://' + host + ':8080/stream?topic=' + topic_name;
    document.getElementById("pro_viewer").src = img_url;
    console.log('Image Viewer is showing: '+ img_url + ' topic');
}

// Drag and Drop
// -----------------
function allowDrop(ev) {
  ev.preventDefault();
}

var state_var = '';
function drag(ev, state) {
  state_var = state; 
}

function drop(ev) {
  ev.preventDefault();
  var drop_id = ev.target.id;
//   if ($(this).find("input id=*clone")){
//     $(document.getElementById(state_var)).appendTo(".dest_list").replaceWith(function() { 
//     return "<li draggable='true' ondragstart='drag(event,this.id)'>" + this.innerHTML + "</li>"; 
// });
//   }
//   else {

  $(document.getElementById(state_var)).clone().appendTo(".dest_list"+drop_id).replaceWith(function() { 
    $(this).find("p").addClass("sclone_p");
    $(this).css('display', 'inline-grid');
    $(this).find("input").removeClass("u-radius-50").css('font-size',' 0rem').css( 'min-width', '0.5rem');
    $(this).find("input").attr("class", state_var + "_clone");
    var del_btn = document.createElement("i");
    del_btn.setAttribute("class","bi bi-x-lg del_btn");
    del_btn.setAttribute("onclick","clear_item(this)");
    $(this).append(del_btn);
    $('#'+state_var + "_clone\*").css('border-radius','0%').css('width','10%').css('margin','-20px').css("height","inherit");
    $(".play_btn").css('margin','0');
    return "<li draggable='true' ondragstart='drag(event,this.id)'>" + this.innerHTML + "</li>"; 
});

}

function auto_run(){
  // click all buttons that end with "clone" in order with a delay of 1 second
    $(".dest_list li input[class$='clone']").each(function(i) {
      $(this).removeClass("active");
      $(this).delay(5000 * i).queue(function() {
        $(this).addClass("active");
        $(this).click();
        $(this).dequeue();
        });

    });
  }

function clean(){
$(".dest_list").empty();
}
function clear_item(item){
$(item).parent().remove();
} 
// End of Drag and Drop
// -----------------


// // Audio Player for the web interface
// // -----------------
function playAudio(input) { 
  $(".play_btn")[0].currentTime = 0;
  
  if ($(input).hasClass("active") ) {
    $(input).removeClass("active");
    
  } 
  else {
    $(".play_btn").removeClass("active");
    $(input).addClass("active"); 
       

    }
  }


// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // <----------------------------------------- END OF INTERFACE FUNCTIONS ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////



// // <----------------------------------------- PAGE LOAD ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

// On page load, below settings will be applied or executed.
window.addEventListener('load', (event) => {
  // get_ip();
  
  sleep(6000).then(() => {  // wait 3 seconds
  console.log('page is fully loaded');
  console.log('Settings have successfully set [android server url = '+android_server_url+'], [Django base url = '+django_base_url+']', '[ROS websocket = '+robot_ws+']');  
});
});

// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // <----------------------------------------- END OF PAGE LOAD ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

  

// // <---------------------------------------------- EMOTION HANDLING SECTION---------------------------------------->	
// // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// -----------------
// Handling the facial imitiator through the server (imitating the user's facial expression through ROS via camera)
// -----------------

// Creating a new Topic for the user's current emotion (subscribing to the /py_exp_publisher topic)
var autoexp_Topic = new ROSLIB.Topic({
  ros : ros,
  name : listen_exp_topic,
  messageType : exp_msg_type
});

autoexp_Topic.subscribe(function(message) { // updating the video file based on the emotion received from the robot (not the user interface)
  console.log('Received message on ' + autoexp_Topic.name + ': ' + message.emotion);
  
  var msgd = message.emotion;
  $.ajax({
    type: "GET",
    url: request_current_exp,
    data: {
      face: msgd
    },
    success: function(response) {
      var sound_url = response.sound_url;
      // Playing the recognized emotion's sound and video file
      document.getElementById("vidsrc").innerHTML = '<source src="'+ response.face_url+'" type="video/mp4">'; 
      document.getElementById("vidsrc").play()
      if (sound_url != 'No assigned sound found') {
        document.getElementById("vidsoundsrc").innerHTML = '<source src="'+ sound_url+'" type="audio/mp3">';
        document.getElementById("vidsoundsrc").play();
        } else {
          document.getElementById("vidsoundsrc").pause();
          document.getElementById("vidsoundsrc").currentTime = 0;
          document.getElementById("vidsoundsrc").innerHTML = '<source src="" type="audio/mp3">';};

      console.log(response);
      var ids =   [response.face_url, sound_url];
      update_exp(ids);  // Returning the id of the button clicked 
                        // and it's relative sound recived as a response from the server
                        //  to be used in the update_exp function.

    }
  });
  document.getElementById("msg").innerHTML = "Auto:"+ msgd;
});

// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ---------------------------------------------- END OF FACIAL IMITATOR ----------------------------------------------
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////



// ---------------------------------------------- EMOTION FILES HANDLING ----------------------------------------------
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

var face_url_id = '';
// Taking in the user's commanded face expression 
// and passing on the url of the video file and its assigned sound's url to update_exp function.
// -----------------
function exp_face(element) {
  var face_url_id = element.id;
  var face_name_val = element.value;
  
  document.getElementById("msg").innerHTML = face_name_val;

  if (element.id == 'auto') {
    auto_imit_val = true;
  }
  
  
  $.ajax({
    type: "GET",
    url: request_current_exp, //Retrieving the requested emotion's sound file from the serever's database
    data: {
      face: face_name_val
    },
    success: function(response) {
      var sound_url = response.sound_url;
      // Playing the requested emotion's sound and video file
      document.getElementById("vidsrc").innerHTML = '<source src="'+ response.face_url+'" type="video/mp4">';
      if (sound_url == 'No assigned sound found' | face_url_id == 'auto') {
        document.getElementById("vidsoundsrc").pause();
        document.getElementById("vidsoundsrc").currentTime = 0;
        document.getElementById("vidsoundsrc").innerHTML = '<source src="" type="audio/mp3">';
      }
      else {
        document.getElementById("vidsoundsrc").innerHTML = '<source src="'+ sound_url+'" type="audio/mp3">';
        document.getElementById("vidsoundsrc").play();
      };
      var ids =   [face_url_id, sound_url];
      update_exp(ids);  // Returning the id of the button clicked 
                        // and it's relative sound recived as a response from the server
                        //  to be used in the update_exp function.
    }
  });

}



// Taking in the user's commanded sound and passing on the url of the sound file to update_exp function.
// -----------------
function exp_sound(element) {
  playAudio(element) // Playing the sound file
  document.getElementById("vidsoundsrc").pause() // Pausing the video file
  document.getElementById("vidsoundsrc").currentTime = 0; // Resetting the video file to the beginning
  var sound_url_val = element.value;
  document.getElementById("msg_sound").innerHTML = sound_url_val; // Displaying the name of the sound file on the web interface
  document.getElementById("vidsoundsrc").innerHTML = '<source src="'+ sound_url_val+'" type="audio/mp3">'; // Updating the sound file
  document.getElementById("vidsoundsrc").play(); // Playing the sound file
  var ids =   [face_url_id, sound_url_val];
  return update_exp(ids); // returning the id of the button clicked to be used in the 'exp' function.

}



// setting the default value for the face expression and sound (neutral)
// -----------------
function set_default_exp(face_name_val='neutral') {

  $.ajax({ // Sending a GET request to the server to retrieve the sound url and face url of the default emotion
    type: "GET",
    url: request_current_exp,
    data: {
      face: face_name_val
    },
    success: function(response) { 
      var sound_url = response.sound_url;
      var face_url = response.face_url;
      // Playing the recognized emotion's sound and video file
      document.getElementById("vidsrc").innerHTML = '<source src="'+ face_url+'" type="video/mp4">'; 
      document.getElementById("vidsrc").play()
      if (sound_url != 'No assigned sound found') {
        document.getElementById("vidsoundsrc").innerHTML = '<source src="'+ sound_url+'" type="audio/mp3">';
        document.getElementById("vidsoundsrc").play();
        } else {
          document.getElementById("vidsoundsrc").pause();
          document.getElementById("vidsoundsrc").currentTime = 0;
          document.getElementById("vidsoundsrc").innerHTML = '<source src="" type="audio/mp3">';};
 
      console.log(response);
      var ids =   [face_url, sound_url];
      update_exp(ids);  // Returning the name of the emotion
                        // and it's relative sound recived as a response from the server
                        //  to be used in the update_exp function.

    }
  });
  document.getElementById("msg").innerHTML = "Default:"+ face_name_val; // Displaying the name of the emotion on the web interface
}


// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // <----------------------------------------- END OF EMOTION FILES HANDLING ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////



// ---------------------------------------------- UPDATING EMOTIONS ----------------------------------------------
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Updating sounds and facial expressions through the Django server and publishing the new emotion to the robot through ROS
// -----------------
function update_exp(ids) {
  // Sending a GET request to the server to set a new emotion
  // -----------------
  $.ajax({
    type: "GET",
    url: android_server_url}); // Sending the request to the android server to let it know that the user has requested a new emotion
  
  $.ajax({
    type: "GET",
    url: publish_new_exp,
    data: {
      // Updating sound and video urls based on returned data from exp_face and exp_sound functions
      face: ids[0],
      sound: ids[1]
    },
    success: function(response) {
      document.getElementById("vidsrc").load(); // Loading the new requested video file
      document.getElementById("vidsrc").play(); // Playing the new requested video file
      
      console.log(response); // logging the response in browser's console
    }
  });

  // Publishing the new emotion to the robot through ROS
  var exp_msg = new ROSLIB.Message({
    emotion : ids[0],
    auto_imit: auto_imit_val,
    action: 'face expression'
   });
   exp_Topic.publish(exp_msg); // Publishing the new emotion to the robot
   auto_imit_val = false; // Resetting the auto_imit_val to false to prevent auto-imitating the emotion
  }
  ids = []; // Resetting the ids array to prevent updating the video file when sound updates
  
// // <--- END OF UPDATE_EXP FUNCTION --->

function get_latest_emotion() { // Getting the latest emotion from the server
  $.ajax({
    type: "GET",
    url: get_latest_emotion_url,
    
    success: function(response) {
      console.log(response);
      var sound_url = response.sound;
      var face_url = response.face;
      // Playing the recognized emotion's sound and video file
      document.getElementById("vidsrc").pause(); // Pausing the video file
      document.getElementById("vidsrc").currentTime = 0; // Resetting the video file to the beginning
      document.getElementById("vidsrc").innerHTML = '<source src="'+ face_url+'" type="video/mp4">'; 
      document.getElementById("vidsrc").load(); // Loading the new requested video file
      document.getElementById("vidsrc").play() // Playing the new requested video file
      if (sound_url != 'No assigned sound found') {
        document.getElementById("vidsoundsrc").innerHTML = '<source src="'+ sound_url+'" type="audio/mp3">'; // Updating the sound file
        document.getElementById("vidsoundsrc").play(); // Playing the sound file
        } else {
          document.getElementById("vidsoundsrc").pause(); // Pausing the sound file
          document.getElementById("vidsoundsrc").currentTime = 0; // Resetting the sound file to the beginning
          document.getElementById("vidsoundsrc").innerHTML = '<source src="" type="audio/mp3">';}; // Updating the sound file
      
    }
  });
}

// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // <----------------------------------------- END OF EMOTION HANDLING ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

// // <----------------------------------------- CSRF Token Handling ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

var csrftoken = $.cookie('csrftoken'); // Getting the CSRF token from the browser's cookies

function csrfSafeMethod(method) {
  // these HTTP methods do not require CSRF protection
  return (/^(GET|HEAD|OPTIONS|TRACE)$/.test(method)); // Returning true if the method is GET, HEAD, OPTIONS, or TRACE
}

$.ajaxSetup({ // Setting up the ajax request to include the CSRF token in the header
  beforeSend: function(xhr, settings) { // Before sending the request
    if (!csrfSafeMethod(settings.type) && !this.crossDomain) { // If the request is not safe and is not cross domain
      xhr.setRequestHeader("X-CSRFToken", csrftoken); // Set the CSRF token in the header
    }
  }
});

// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // <----------------------------------------- END OF CSRF Token Handling ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////


// // <----------------------------------------- PARROT ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

function parr_b(element) {
  document.getElementById("pb_msg").innerHTML = element.value; 
  dta=JSON.stringify({
    id: element.id,
    tag: element.id,
    '_token': csrf
  });
  $.ajax({
    type: "POST",
    contentType : 'application/json',
    url: 'parrot/',
    data: dta,
    success: function(response) {
      // Playing the new requested video and sound file
      console.log(response);
    },
    error: function(response) {
    console.log(response);// logging the response in browser's console
}});
}
function parr_r(element) {
  document.getElementById("pr_msg").innerHTML = element.value;
}

// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // <----------------------------------------- END OF PARROT ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // <----------------------------------------- TEXT-TO-SPEECH ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

function tts() {
  var text = document.getElementById("tts_text").value;

  var text_to_speech_client = new ROSLIB.Service({
      ros : ros,
      name : tts_service,
      serviceType : tts_srv_type
    });
  
    var request = new ROSLIB.ServiceRequest({
      text : text,
    });
  
    text_to_speech_client.callService(request, function(result) {
      console.log('Service called successfully. \n The generated sound will be played on the robot, not on the web interface.');
    });
}


// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // <----------------------------------------- END OF TEXT-TO-SPEECH ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////


// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // <----------------------------------------------- MOTION HANDLING --------------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

// // -----------------
// // Listening to the /cmd_vel topic to get the robot's current velocity commanded through keyboard teleoperation.
// // -----------------

var motion_Topic = new ROSLIB.Topic({
  ros : ros,
  name : listen_motion_topic,
  messageType : motion_msg_type
});


motion_Topic.subscribe(function(message) {
  console.log('Received message on ' + exp_Topic.name + ': ' + message.linear.x);
  linear_x = message.linear.x;
  linear_y = message.linear.y;
  angular_z = message.angular.z
  
});

// -----------------
// Creating new Topic for dynamixel movement commands
// -----------------
var dyna_Topic = new ROSLIB.Topic({
  ros : ros,
  name : dyna_topic,
  messageType : dyna_msg_type
});

var exp_Topic = new ROSLIB.Topic({
  ros : ros,
  name : publish_exp_topic,
  messageType : exp_msg_type
});



// // -----------------
// // Publishing manual movement commands from the user interface(not from the keyboard) on /cmd_vel_web topic
// // -----------------

function get_speed(){
var speed = document.getElementById("speed").value;
return speed;
}

function get_degrees(){
var degrees = document.getElementById("degrees").value;
return degrees;
}

// -----------------
// The function that will be called from the move_keys function when the user clicks on the move buttons
// -----------------
function move(cw, joint) { 
  var cmd_vel_listener = new ROSLIB.Topic({
    ros : ros,
    name : dyna_topic,
    messageType : dyna_msg_type
  });
  
  cmd_vel_listener.subscribe(function(message) {
    console.log('Received message on ' + dyna_Topic.name + ' for ' + message.joint);
  });
  angular = get_speed();
  degree = get_degrees()*cw;
  console.log(degree, joint)
  if (joint == 'reset'){
    degree = 0;
    console.log('Resetting all joints to 0');

  }
  else {
    // the position is in degrees and is how much the joint will move from its current position
    console.log('Moving '+joint+' '+degree+' degress'+ ' with angular speed of '+angular+' degrees/sec');
  }

// Creating a message of the type DynaTwist to be published on the /cmd_vel/dyna topic
var twist = new ROSLIB.Message({
  linear: {
    x: 0,
    y: 0,
    z: 0
  },
  angular: {
    x: parseFloat(angular),
    y: 0,
    z: 0
  }
});

var dyna_twist = new ROSLIB.Message({
  speed: twist,
  position: parseInt(degree),
  joint: joint
});
cmd_vel_listener.publish(dyna_twist);
console.log(dyna_twist)

var motion_Topic = new ROSLIB.Topic({
  ros : ros,
  name : listen_dyna_status_topic,
  messageType : dyna_status_msg_type
});

var current_pos;
var current_id;
motion_Topic.subscribe(function(message) {// listening to the /cmd_vel_web topic to get the robot's current joint positions through the web interface.
  console.log('Received message on ' + listen_dyna_status_topic + ': ' + message.joint + ':' + message.position);
  current_pos = message.position;
  current_id = message.joint;
  document.getElementById('current_id').innerHTML = current_id;
  document.getElementById('current_pos').innerHTML = current_pos;
}); 
}


// -----------------
// The function that will be called when the user clicks on the move buttons
// -----------------
function move_keys(joint,pos){ // joint: head, neck, rhand, lhand | pos: up, down, left, right
  if (joint == 'reset'){
    move(1,'reset');
  }
  else if (joint == 'head'){
    if (pos == 'up'){
      move(1,'head');
    }
    else if (pos == 'down'){
      move(-1,'head');
    }
  }
  else if (joint == 'neck'){
    if (pos == 'left'){
      move(-1,'neck');
    }
    else if (pos == 'right'){
      move(1,'neck');
    }
  }
  else if (joint == 'rhand'){
    if (pos == 'up'){
      move(1,'rhand');
    }
    else if (pos == 'down'){
      move(-1,'rhand');
    }
  }
  else if (joint == 'lhand'){
    if (pos == 'up'){
      move(1,'lhand');
    }
    else if (pos == 'down'){
      move(-1,'lhand');
    }
  }
}

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // <----------------------------------------- END OF MOTION HANDLING ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////


function submitWizardForm(data) {
  dtaa = JSON.stringify({
    "csrfmiddlewaretoken": csrf,
    data,
  });
  $.ajax({
    type: "POST",
    contentType : 'application/json',
    url: 'setup/',
    data: dtaa,
    success: function(response) {
      // Playing the new requested video and sound file
      console.log(response);
    },
    error: function(response) {
    console.log(response);// logging the response in browser's console
}});}


// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // <----------------------------------------- SERVICES ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

      // // Calling a service
      // // -----------------
      // var speech_to_text_client = new ROSLIB.Service({
      //   ros : ros,
      //   name : '/web_speech_to_text',
      //   serviceType : 'infrastructure/EmoProb'
      // });
    
      // var request = new ROSLIB.ServiceRequest({
      //   a : 1,
      //   b : 2
      // });
    
      // speech_to_text_client.callService(request, function(result) {
      //   console.log('Result for service call on '
      //     + speech_to_text_client.name
      //     + ': '
      //     + result.sum);
      // });

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // <----------------------------------------- END OF SERVICES ----------------------------------------->
// // ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

// -----------------
// Get the msg type of the specified topic to publish on
// -----------------
function setrostopic(topic_name) {
  var msg_type;
  // send a GET request to the Django server to set the topic name, and retrieve the topic type (aka. msg type)
  $.ajax({
    type: "GET",
    url: get_msg_type_url,
    data: {
      topic: topic_name
    },
    success: function(response) {
      console.log(response);
      msg_type = response.msg_type;
      
      var ros_topic = new ROSLIB.Topic({
        ros: ros,
        name: topic_name,
        messageType: msg_type // Set the message type of the topic
      });
      
      ros_topic.subscribe(function(message) {
        const fields = Object.keys(message); // Get the fields of the message type
        console.log(fields);
        const form = document.getElementById('messageForm');
        
        fields.forEach((field) => { // Create an input element for each field of the message type
          const input = document.createElement('input');
          input.setAttribute('type', 'text');
          input.setAttribute('name', field);
          input.setAttribute('placeholder', field);

          form.appendChild(input);
          const bt = document.getElementById('pub');
          bt.style.display = 'block';
        });

        publish_msg_form(ros_topic); // Publish the message when the form is submitted
      });
    }
  });
}

// -----------------
// Publish the message when the form is submitted
// -----------------
function publish_msg_form(topic) {
  const msgform = document.getElementById('messageForm');
  msgform.addEventListener('submit', function(event) { // Publish the message when the form is submitted
    event.preventDefault();

    const formData = new FormData(msgform);
    const message = new ROSLIB.Message();
    
    for (const pair of formData.entries()) {
      message[pair[0]] = pair[1];
    }

    topic.publish(message);
  });
}

function echorostopic(topic_name) {
  var msg_type;
  // send a GET request to the Django server to set the topic name, and retrieve the topic type (aka. msg type)
  $.ajax({
    type: "GET",
    url: get_msg_type_url,
    data: {
      topic: topic_name
    },
    success: function(response) {
      console.log(response);
      msg_type = response.msg_type;
      
      var ros_topic = new ROSLIB.Topic({ 
        ros: ros,
        name: topic_name,
        messageType: msg_type // Set the message type of the topic
      });

      ros_topic.subscribe(function(message) {
        // add a new line to the topic's div with the message
        const div = document.getElementById('topics');
        const p = document.createElement('p');
        p.innerHTML = JSON.stringify(message);
        div.appendChild(p);
        p.innerHTML = '------------------ <br>';
        div.appendChild(p);
        
      });
    }
  });
}