// Create events for the sensor readings
if (!!window.EventSource) {
  var source = new EventSource('/events');

  source.addEventListener('open', function(e) {
    console.log("Events Connected");
  }, false);

  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  }, false);

  source.addEventListener('servo_readings', function(e) {
    //console.log("gyro_readings", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("theta1").innerHTML = obj.theta1;
    document.getElementById("theta2").innerHTML = obj.theta2;
    document.getElementById("theta3").innerHTML = obj.theta3;
    document.getElementById("theta4").innerHTML = obj.theta4;
    document.getElementById("theta5").innerHTML = obj.theta5;

    // Change cube rotation after receiving the readings
    cube.rotation.z = obj.theta1;
    renderer.render(scene, camera);
  }, false);



  source.addEventListener('joint_readings', function(e) {
    console.log("joint_readings", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("joint1").innerHTML = obj.joint1;
    document.getElementById("joint2").innerHTML = obj.joint2;
    document.getElementById("joint3").innerHTML = obj.joint3;
    document.getElementById("joint4").innerHTML = obj.joint4;
    document.getElementById("gripper1").innerHTML = obj.gripper1;
  }, false);

  source.addEventListener('pose_readings', function(e) {
    console.log("pose_readings", e.data);
    var obj = JSON.parse(e.data);
    document.getElementById("posX").innerHTML = obj.posX;
    document.getElementById("posY").innerHTML = obj.posY;
    document.getElementById("posZ").innerHTML = obj.posZ;
    document.getElementById("pitch").innerHTML = obj.pitch;
    }, false);
}



