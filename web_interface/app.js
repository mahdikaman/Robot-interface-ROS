const express = require('express')
const path = require('path');
var bodyParser = require('body-parser');
const rclnodejs = require('rclnodejs');

// Create Express app
const app = express()
app.use(bodyParser.json())
// app.use(bodyParser.urlencoded());
app.use(express.static(__dirname + '/public'));

// Init rclnodejs for ROS2
rclnodejs.init()

// Create node for the web interface
const node = new rclnodejs.Node('iths_web_interface_node');
// const client = new ITHSActionClient(node);
const robot = node.createPublisher('std_msgs/msg/Float64MultiArray', '/forward_position_controller/commands');
rclnodejs.spin(node)

let head = 0;
let left_arm = 0;
let right_arm = 0;

const publishPositions = () => robot.publish({
  layout: {
    dim: [
      {
        label: 'head',
        size: 1,
        stride: 0,
      },
      {
        label: 'left_arm',
        size: 1,
        stride: 1,
      },
      {
        label: 'right_arm',
        size: 1,
        stride: 2,
      },
    ],
    data_offset: 0,
  },
  data: [head, left_arm, right_arm],
});

// Root node
app.get('/', function (req, res) {
  res.sendFile(path.join(__dirname, '/index.html'));
});

// Move the left arm
app.post('/move_left_arm', function (req, res) {
  node.getLogger().info('Moving left arm');
  left_arm = req.body.position;
  publishPositions();
  res.sendStatus(200);
});

// Move the left arm
app.post('/move_head', function (req, res) {
  node.getLogger().info('Moving head');
  head = req.body.position;
  publishPositions();
  res.sendStatus(200);
});

// Move the left arm
app.post('/move_right_arm', function (req, res) {
  node.getLogger().info('Moving right arm');
  right_arm = req.body.position;
  publishPositions();
  res.sendStatus(200);
});

app.get('/reset', function (req, res) {
  node.getLogger().info('reset');
  right_arm = 0;
  left_arm = 0; 
  head = 0;
  publishPositions();
  res.sendStatus(200);
});

app.get('/start_rosbag_recording', function (req, res) {
  node.getLogger().info('start');
  res.sendStatus(200);
});

app.get('/stop_rosbag_recording', function (req, res) {
  node.getLogger().info('start');
  res.sendStatus(200);
});


// Start the Express server
const server = app.listen(3000, () => console.log('Server running on port 3000!'))

// Try to exit gracefully
process.on('SIGTERM', () => {
  server.close(() => {
    console.log('Process terminated');
    rclnodejs.shutdown();
  });
});

process.on('SIGINT', function() {
  server.close(() => {
    console.log('Process terminated');
    rclnodejs.shutdown();
  });
});
