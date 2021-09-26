// Node.js socket server script
const net = require('net');
var Threads= require('webworker-threads');
const spawn = require('child_process').spawn;
var worker;
const server = net.createServer((socket) => {
    socket.on('data', (data) => {
    console.log(data.toString());
    //var worker = new Threads.Worker('launch.js');
    if (data.toString() == 'start'){
        worker = spawn('./test.sh', {detached: true});
        //worker = new Threads.Worker('launch.js');
    }
    else if (data.toString() == 'kill'){
        process.kill(-worker.pid);
        //postMessage('Happy Birthday');
        console.log("I'll close the platform!");
    }
    });
    socket.write('SERVER: Hello! This is server speaking.<br>');
    socket.end('SERVER: Closing connection now.<br>');
}).on('error', (err) => {
    console.error(err);
});
// Open server on port 9898
server.listen(9898, () => {
    console.log('opened server on', server.address().port);
});