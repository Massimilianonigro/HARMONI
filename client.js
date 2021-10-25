const io = require("socket.io-client");
const find = require('local-devices');

var robot_ip;


find().then(devices => {
    devices
    console.log(devices); 
    devices.forEach(device => {
        if (device['mac'] == '00:d0:ca:01:f8:53') {
            robot_ip = device["ip"];
        }
    }
    )
    console.log(robot_ip);
    /*
    [
      { name: '?', ip: '192.168.0.10', mac: '...' },
      { name: '...', ip: '192.168.0.17', mac: '...' },
      { name: '...', ip: '192.168.0.21', mac: '...' },
      { name: '...', ip: '192.168.0.22', mac: '...' }
    ]
    */

    var execSync = require('child_process').execSync;
    var exec = require('child_process').exec, child;

    var addresses;

    var local_url;

    addresses = execSync("hostname -I", option = {encoding : 'utf8'}, function(error, stdout, stderr){ addresses = stdout; });


    local_url = addresses.split(" ")[3];
    console.log(local_url)

    const socket = io("https://misty.I3lab.group/", {
        path: "/wss/socket.io",
        auth: {
            token: "123",
            ip : local_url
        },
    });

    console.log("ok here");

    socket.on("connect",  () => {
        console.log("connected");
    })

    socket.on('start', function(msg) {
        console.log("I'll start the platform");
        console.log('ip ' + robot_ip + ' ' + msg['activityType'] + ' ' + msg['activity'] + ' ' + msg['level3'] + ' ' + msg['level4']+ ' ' + local_url)
        child = exec('./activity_launch.sh ' + robot_ip + ' ' + msg['activityType'] + ' ' + msg['activity'] + ' ' + msg['level3'] + ' ' + msg['level4'] + ' ' + local_url,
        function (error, stdout, stderr) {
            console.log('stdout: ' + stdout);
            console.log('stderr: ' + stderr);
            if (error !== null) {
                console.log('exec error: ' + error);
            }
        }
        );
    })

    socket.on('kill', function(msg) {
        process.kill(-worker.pid);
        console.log("I'll close the platform!");
    })
  })

