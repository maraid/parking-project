
var SerialPort = require('serialport');
var port = new SerialPort('/dev/ttyUSB0', {
  autoOpen: false,
  baudRate: 9600
});
 
port.open(function (err) {
  if (err) {
    return console.log('Error opening port: ', err.message);
  }
  
});
 
// The open event is always emitted
port.on('open', function() {
  // open logic
  console.log('port open!')
  //console.log(port.read());
  port.write(Buffer.from('B' + '\r' + '\n'));
});

port.on('readable', function () {
  console.log('Data:', port.read().toString());
});


var express = require('express');
const bodyParser = require('body-parser')
var app = express();
var path = require('path');

app.use(bodyParser.urlencoded({ extended: false }))

// viewed at http://localhost:8080
app.get('/', function(req, res) {
    res.sendFile(path.join(__dirname + '/index.html'));
});

app.post("/control", function (req, res) {
	var dir = req.body.direction
    console.log(dir);
    res.set('Content-Type', 'text/plain')
    res.send('OK')
    
    if(dir === 'U') {
		port.write(Buffer.from('D,10,10' + '\r' + '\n'));
	} else if(dir === 'D') {
		port.write(Buffer.from('D,-10,-10' + '\r' + '\n'));	
	} else if(dir === 'L') {
		port.write(Buffer.from('D,10,-10' + '\r' + '\n'));	
	} else if(dir === 'R') {
		port.write(Buffer.from('D,-10,10' + '\r' + '\n'));	
	} else {
		//console.log('other')
		port.write(Buffer.from('D,0,0' + '\r' + '\n'));	
	}
    //console.log(res);
});

app.listen(8080);
