// Create a server object
const shell = require('shelljs')

var cleanExit = function() { process.exit(this) };
process.on('SIGINT', cleanExit); 

shell.exec('./test.sh');