var SerialPort = require('serialport');
var current='';
var port = new SerialPort('/dev/ttyACM0', {
    baudRate: 9600
  });

port.on('data',gotData);
port.on('error',gotError);

function gotError(err){
    debugger;
}

function gotData(data){
    var line = data.toString('utf8');
    var idx=line.indexOf('}');
    if(idx==-1)
        current+=line;
    else{
        current+=line.substring(0,idx+1);

        var output=JSON.parse(current);
        current=line.substring(idx+1);
        console.log(output);
    }

}