document.addEventListener('DOMContentLoaded', function(){
    var head_now = 0;
    var neck_now = 0;
    var ros = new ROSLIB.Ros({
        url : 'ws://localhost:9090'
    });

    ros.on('connection', function() {
        console.log('Joystick connected to websocket server.');
    });

    var cmd_vel_listener = new ROSLIB.Topic({
        ros : ros,
        name : dyna_topic,
        messageType : dyna_msg_type
        });

	var joystick = document.getElementById("joystick"),
			    knob = document.getElementById("knob"),
			target_x = joystick.clientWidth/2-knob.clientWidth/2,
			target_y = joystick.clientHeight/2-knob.clientHeight/2;

	var panSpan  = document.getElementById("panValue"),
			tiltSpan = document.getElementById("tiltValue");

	knob.style.webkitTransform = "translate("+target_x+"px, "+target_y+"px)";

	// update the position attributes
	var target = document.getElementById("knob");
	updatePositionAttributes(target,target_x,target_y);
    
	// target elements with the "draggable" class
	interact('.draggable')
    .draggable({
        inertia: false,
        // keep the element within the area of its parent
        restrict: {
            restriction: "parent",
            endOnly: false,
            elementRect: { top: 1, left: 0, bottom: 0, right: 1 }
        },
        onmove: dragMoveListener,
        onend: function (event) {
            move(panSpan.innerHTML, tiltSpan.innerHTML);
            var target = event.target;
            TweenLite.to(target, 0.2, {ease: Back.easeOut.config(1.7), "webkitTransform":"translate("+target_x+"px, "+target_y+"px)"});
            updatePositionAttributes(target,target_x,target_y);
            panSpan.innerHTML = 0;
            tiltSpan.innerHTML = 0;
        }
    });
    
    function dragMoveListener (event) {
        var target = event.target,
        // keep the dragged position in the data-x/data-y attributes
        x = parseInt(parseInt(target.getAttribute('data-x')) || 0) + parseInt(event.dx*3);
        y = parseInt(parseInt(target.getAttribute('data-y')) || 0) + parseInt(event.dy*3);
        
        // translate the element
        target.style.webkitTransform = target.style.transform = 'translate(' + x + 'px, ' + y + 'px)';
        updatePositionAttributes(target,x,y);
        
        // update text display
        panSpan.innerHTML = parseInt(x-joystick.clientWidth/4);
        tiltSpan.innerHTML = parseInt(y-joystick.clientHeight/4);
    }
    
    function updatePositionAttributes(element,x,y){
        target.setAttribute('data-x', x);
        target.setAttribute('data-y', y);
        
	}

    function move(head_deg, neck_deg) { 
        var dyna_topic = '/cmd_vel/dyna';
        var dyna_msg_type = 'infrastructure/DynaTwist';
        var angular = 60;
        var head = head_deg;
        var neck = neck_deg*-1;
        if (head > -10 && head < 10) {
            head = 0;
        }
        
        if (neck > -10 && neck < 10) {
            neck = 0;
        }
        
        head_now = parseFloat(head_now) + parseFloat(head);
        neck_now = parseFloat(neck_now) + parseFloat(neck);

        if (head_now<-150) {
            head_now = -150
        }
        if (head_now>150) {
            head_now = 150
        }

        if (neck_now<-150) {
            neck_now = -150
        }
        if (neck_now>150) {
            neck_now = 150
        }

        head_now = parseFloat(head_now) + parseFloat(head);
        neck_now = parseFloat(neck_now) + parseFloat(neck);
        // the position is in degrees and is how much the joint will move from its current position
        console.log('Moving head to position'+head_now+' '+' and neck to '+ neck_now+' with angular speed of '+angular+' degrees/sec');
        
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
        
        var dyna_twist_head = new ROSLIB.Message({
            speed: twist,
            position: parseInt(head_now),
            joint: 'head'
        });
        
        

        var dyna_twist_neck = new ROSLIB.Message({
            speed: twist,
            position: parseInt(neck_now),
            joint: 'neck'
        });
        console.log(dyna_twist_neck.speed,dyna_twist_neck.position , dyna_twist_head.speed, dyna_twist_head.position);
        cmd_vel_listener.publish(dyna_twist_head);
        cmd_vel_listener.publish(dyna_twist_neck);
        
        return false;
    }

});