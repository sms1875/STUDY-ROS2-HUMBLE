import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'dart:convert';
import 'dart:async';

void main() => runApp(TurtleControllerApp());

class TurtleControllerApp extends StatelessWidget {
  const TurtleControllerApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: TurtleController(),
    );
  }
}

class TurtleController extends StatefulWidget {
  const TurtleController({super.key});

  @override
  _TurtleControllerState createState() => _TurtleControllerState();
}

class _TurtleControllerState extends State<TurtleController> {
  WebSocketChannel? channel; // 웹소켓 채널을 nullable로 변경
  String serverAddress = 'ws://localhost:9090'; // 기본 서버 주소
  String connectionStatus = "Disconnected";
  String movementLog = "";
  String selectedTurtle = 'turtle1';
  Timer? _timer;

  List<String> turtles = ['turtle1'];

  @override
  void dispose() {
    _timer?.cancel();
    channel?.sink.close(); // 웹소켓 연결 해제
    super.dispose();
  }

  void _connectWebSocket() {
    try {
      channel = WebSocketChannel.connect(Uri.parse(serverAddress));
      setState(() {
        connectionStatus = "Connecting...";
      });

      channel!.stream.listen((data) {
        setState(() {
          connectionStatus = "Connected";
        });
        print("Connected to ROS WebSocket");
      }, onError: (error) {
        setState(() {
          connectionStatus = "Connection Error";
        });
        print("WebSocket Error: $error");
      }, onDone: () {
        setState(() {
          connectionStatus = "Disconnected";
        });
        print("WebSocket Disconnected");
      });

      _timer = Timer.periodic(const Duration(seconds: 1), (timer) {
        sendKeepAliveMessage();
      });
    } catch (e) {
      setState(() {
        connectionStatus = "Connection Error";
        print("Failed to connect: $e");
      });
    }
  }

  void _disconnectWebSocket() {
    if (channel != null) {
      channel!.sink.close();
      setState(() {
        connectionStatus = "Disconnected";
        _timer?.cancel();
      });
      print("Disconnected from ROS WebSocket");
    }
  }

  // 선택된 거북이에 대한 cmd_vel 토픽을 광고하는 함수
  void _advertiseCmdVel(String turtleName) {
    if (channel == null) return;
    Map<String, dynamic> advertiseMsg = {
      "op": "advertise",
      "topic": "/$turtleName/cmd_vel",
      "type": "geometry_msgs/Twist"
    };

    channel!.sink.add(jsonEncode(advertiseMsg));
    print("Advertising /$turtleName/cmd_vel");
  }

  // 거북이를 생성하는 서비스 요청
  void createTurtle() {
    if (channel == null) return;
    Map<String, dynamic> serviceMsg = {
      "op": "call_service",
      "service": "/spawn",
      "args": {
        "x": 5.0,
        "y": 5.0,
        "theta": 0.0
      }
    };

    channel!.sink.add(jsonEncode(serviceMsg));

    // 새 거북이 추가 (turtle2, turtle3 등)
    setState(() {
      String newTurtle = 'turtle${turtles.length + 1}';
      turtles.add(newTurtle);
      selectedTurtle = newTurtle;
      _advertiseCmdVel(selectedTurtle);
    });
  }

  // 거북이를 제거하는 서비스 요청
  void killTurtle() {
    if (channel == null || selectedTurtle == 'turtle1') {
      _logMovement("Cannot kill turtle1");
      return;
    }

    Map<String, dynamic> serviceMsg = {
      "op": "call_service",
      "service": "/kill",
      "args": {
        "name": selectedTurtle
      }
    };

    channel!.sink.add(jsonEncode(serviceMsg));
    _logMovement("Killed $selectedTurtle");

    setState(() {
      turtles.remove(selectedTurtle);
      selectedTurtle = turtles.first; // turtle1로 되돌림
    });
  }

  // keep-alive 메시지를 보냄
  void sendKeepAliveMessage() {
    if (channel == null) return;
    Map<String, dynamic> message = {
      "op": "publish",
      "topic": "/$selectedTurtle/cmd_vel",
      "msg": {
        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
      }
    };
    channel!.sink.add(jsonEncode(message));
  }

  // 선택된 거북이를 이동시키는 함수
  void moveTurtle(String direction) {
    if (channel == null) return;
    Map<String, dynamic> message = {
      "op": "publish",
      "topic": "/$selectedTurtle/cmd_vel",
      "msg": {
        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
      }
    };

    String action = "";

    switch (direction) {
      case 'up':
        message['msg']['linear']['x'] = 2.0;
        action = "Moving Forward";
        break;
      case 'down':
        message['msg']['linear']['x'] = -2.0;
        action = "Moving Backward";
        break;
      case 'left':
        message['msg']['angular']['z'] = 2.0;
        action = "Turning Left";
        break;
      case 'right':
        message['msg']['angular']['z'] = -2.0;
        action = "Turning Right";
        break;
    }

    channel!.sink.add(jsonEncode(message));
    _logMovement("$action on $selectedTurtle");
  }

  void _logMovement(String action) {
    setState(() {
      movementLog = "Action: $action\n$movementLog";
    });
    print("Turtle Action: $action");
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Turtle Controller'),
      ),
      body: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: <Widget>[
          // 서버 주소 입력 필드
          TextField(
            onChanged: (value) {
              serverAddress = value;
            },
            decoration: InputDecoration(
              labelText: "WebSocket Server Address",
              hintText: "ws://localhost:9090",
              border: OutlineInputBorder(),
            ),
          ),
          const SizedBox(height: 20),
          // Connect 버튼
          ElevatedButton(
            onPressed: _connectWebSocket,
            child: const Text('Connect'),
          ),
          // Disconnect 버튼
          ElevatedButton(
            onPressed: _disconnectWebSocket,
            child: const Text('Disconnect'),
          ),
          Text("Connection Status: $connectionStatus"),
          const SizedBox(height: 20),
          DropdownButton<String>(
            value: selectedTurtle,
            items: turtles.map((String turtle) {
              return DropdownMenuItem<String>(
                value: turtle,
                child: Text(turtle),
              );
            }).toList(),
            onChanged: (String? newValue) {
              setState(() {
                selectedTurtle = newValue!;
                _advertiseCmdVel(selectedTurtle);
              });
            },
          ),
          const SizedBox(height: 20),
          ElevatedButton(
            onPressed: () => moveTurtle('up'),
            child: const Text('Up'),
          ),
          Row(
            mainAxisAlignment: MainAxisAlignment.center,
            children: <Widget>[
              ElevatedButton(
                onPressed: () => moveTurtle('left'),
                child: const Text('Left'),
              ),
              ElevatedButton(
                onPressed: () => moveTurtle('right'),
                child: const Text('Right'),
              ),
            ],
          ),
          ElevatedButton(
            onPressed: () => moveTurtle('down'),
            child: const Text('Down'),
          ),
          const SizedBox(height: 20),
          ElevatedButton(
            onPressed: createTurtle,
            child: const Text('Create Turtle'),
          ),
          ElevatedButton(
            onPressed: killTurtle,
            child: const Text('Kill Turtle'),
          ),
          const SizedBox(height: 20),
          const Text(
            "Movement Log:",
            style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold),
          ),
          Expanded(
            child: SingleChildScrollView(
              child: Text(movementLog),
            ),
          ),
        ],
      ),
    );
  }
}
