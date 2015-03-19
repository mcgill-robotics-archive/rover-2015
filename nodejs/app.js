/*
 * Express and middleware
 */
var $ = require('jquery');
 console.log("app.js");
var ROSLIB = require('roslib');
//var MJPEGCANVAS = require('./include');
console.log(ROSLIB);
var express = require('express');
var stylus = require('stylus');
var path = require('path');
var favicon = require('serve-favicon');
var logger = require('morgan');
var cookieParser = require('cookie-parser');
var bodyParser = require('body-parser');
var debug = require('debug')('ToolsFrontend');
var http = require('http');
var socketio = require('socket.io');

var ros = new ROSLIB.Ros({
         url : 'ws://localhost:9090'
       })

ros.on('connection', function() {
      console.log('Connected to websocket server.');
       });

ros.on('error', function(error) {
     console.log('Error connecting to websocket server: ', error);
    });


var listener = new ROSLIB.Topic({
  ros : ros,
  name : '/listener',
  messageType : 'std_msgs/String'
});


// Then we add a callback to be called every time a message is published on this topic.
listener.subscribe(function(message) {
  console.log('Received message on ' + listener.name + ': ' + message.data);
  // If desired, we can unsubscribe from the topic as well.
  
});

  var posedata = [];

 // First, we create a Topic object with details of the topic's name and message type.
  var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/cmd_vel',
    messageType : 'geometry_msgs/Twist'
  });

  var poseListener = new ROSLIB.Topic({
    ros : ros,
    name : '/pose',
    messageType : 'geometry_msgs/Pose'
  });
  
  cmdVel.subscribe(function(message) {
  console.log('Received message on ' + cmdVel.name + ': ' + message);
  // If desired, we can unsubscribe from the topic as well.

  
  });

  poseListener.subscribe(function(message) {
  console.log('Received message on ' + poseListener.name + ': ' + message.orientation.x);
  // If desired, we can unsubscribe from the topic as well.
  posedata = [message.orientation.x,message.orientation.y,message.orientation.z,message.orientation.w];
  
  });



 //* Modules
 
var compiler = require('./modules/compile');

/*
 * Get routes
 */
var routes = require('./routes/index');

/*
 * Server vars
 */
var usersConnected = 0;

/*
 * Server init
 */
var app = express();
var httpServer = http.Server(app);
var io = require('socket.io').listen(httpServer);

/*
 *Compile step
 */
compiler('public/components', app.get('env') === 'development');

/*
 * View Engine Setup
 */
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'jade');

/*
 * Style Engine Setup
 */
app.use(stylus.middleware(path.join(__dirname, 'public')));

/*
 * Favicon
 */
app.use(favicon(__dirname + '/public/img/favicon.gif'));

/*
 * Log requests
 */
app.use(logger('dev'));

/*
 * Parser middleware
 */
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({
    extended: false
}));
app.use(cookieParser());

/*
 * Append directories to base path
 */

app.use(express.static(path.join(__dirname, 'public')));
app.use(express.static(path.join(__dirname, 'bower_components')));

/*
 * Socket.io connection
 */
var userIO = function(socket) {
    console.log(++usersConnected + ' users connected');
    socket.on('disconnect', function() {
        console.log(--usersConnected + ' users connected');
    });
};
io.on('connection', userIO);


function sendPose(){
  io.sockets.emit('sendPose', posedata);
}
setInterval(sendPose, 100);
function sendTime() {
    io.sockets.emit('time', { time: new Date().toLocaleTimeString() });
}

// Send current time every 1 secs
setInterval(sendTime, 1000);


/*
 * Timeout
 */
app.use(function(req, res, next) {
    res.setTimeout(120000, function() {
        console.log('Request has timed out.');
        res.send(408);
    });
    next();
});

/*
 * Server set routes
 */
app.use('/', routes);


// catch 404 and forward to error handler
app.use(function(req, res, next) {
    var err = new Error('Not Found');
    err.status = 404;
    next(err);
});

// error handlers

// development error handler
// will print stacktrace
if (app.get('env') === 'development') {
    app.use(function(err, req, res, next) {
        res.status(err.status || 500);
        res.render('errorDebug', {
            message: err.message,
            error: err
        });
    });
}

// production error handler
// no stacktraces leaked to user
app.use(function(err, req, res, next) {
    res.status(err.status || 500);
    res.render('error', {
        title: 'Error'
    });
});

app.set('port', process.env.PORT || 8080);

var server = httpServer.listen(app.get('port'), function() {
    debug('Express server listening at address ' + server.address().address + ' on port ' + server.address().port);
});