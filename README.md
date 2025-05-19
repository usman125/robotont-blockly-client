# Robotont Blockly Web Application

=========

This code contains a Blockly based web application to control Robotont. This repo can be used to set up a local web application running Blockly as the visual programming language and translate the blocks into Python ROS executable nodes.

### How to use?

```
$ git clone
$ npm install
$ node server.js
```

Visit `http://localhost:8080/`, you can assign a different port in `server.js`

### Available Block examples

<img src="block_example_visuals/ros_connection.jpg" width="200"/>

This Blockly block has the following JavaScript definition

```javascript
Blockly.Blocks["ros_connection"] = {
  init: function () {
    this.appendDummyInput("DUMMY")
      .setAlign(Blockly.inputs.Align.RIGHT)
      .appendField("CONNECT TO ROS");
    this.appendStatementInput("ros_connection");
    this.setTooltip("set ros node parameters");
    this.setHelpUrl("none");
    this.setColour(15);
  },
};
```

A subsequent python code generation code for this block

```javascript
python.pythonGenerator.forBlock["ros_connection"] = function (block) {
  let code = ``;
  return code;
};
```

<img src="block_example_visuals/robot_move.jpg" width="200"/>

This Blockly block has the following JavaScript definition

```javascript
Blockly.Blocks["robotont_move"] = {
  init: function () {
    this.appendDummyInput()
      .appendField("Draw")
      .appendField(
        new Blockly.FieldDropdown([
          ["RECTANGLE", "RECTANGLE"],
          ["CIRCLE", "CIRCLE"],
          ["STRAIGHT LINE", "STRAIGHT_LINE"],
        ]),
        "DIRECTION"
      )
      .appendField("repeat")
      .appendField(new Blockly.FieldNumber(0, 0, 3, 1), "SPEED")
      .appendField("times.");
    this.setPreviousStatement(true);
    this.setNextStatement(true);
    // this.setOutput(true, null);
    this.setColour(230);
    this.setTooltip("Move robot forward/backward");
  },
};
```

A subsequent python code generation code for this block

```javascript
python.pythonGenerator.forBlock["robotont_move"] = function (block) {
  const code = ``;
  return code;
};
```

These blocks definitions given below have an advanced use, if we want to change the scaffoled class name, node name, or publisher name.

<img src="block_example_visuals/set_class_name.jpg" width="200"/>

This Blockly block has the following JavaScript definition

```javascript
Blockly.Blocks["class_name"] = {
  init: function () {
    this.appendDummyInput("class name")
      .setAlign(Blockly.inputs.Align.RIGHT)
      .appendField("class name")
      .appendField(new Blockly.FieldTextInput(""), "className");
    this.setNextStatement(true, null); // Can connect to blocks after it
    this.setPreviousStatement(true, null); // Can connect to blocks before it
    this.setTooltip("set class name");
    this.setHelpUrl("none");
    this.setColour(120);
  },
};
```

A subsequent python code generation code for this block

```javascript
python.pythonGenerator.forBlock["class_name"] = function (block) {
  const code = ``;
  return code;
};
```

<img src="block_example_visuals/set_node_name.jpg" width="200"/>

This Blockly block has the following JavaScript definition

```javascript
Blockly.Blocks["node_name"] = {
  init: function () {
    this.appendDummyInput("node name")
      .setAlign(Blockly.inputs.Align.RIGHT)
      .appendField("node name")
      .appendField(new Blockly.FieldTextInput(""), "nodeName");
    this.setNextStatement(true, null); // Can connect to blocks after it
    this.setPreviousStatement(true, null); // Can connect to blocks before it

    this.setTooltip("set node name");
    this.setHelpUrl("none");
    this.setColour(120);
  },
};
```

A subsequent python code generation code for this block

```javascript
python.pythonGenerator.forBlock["node_name"] = function (block) {
  const code = ``;
  return code;
};
```

<img src="block_example_visuals/set_publisher_name.jpg" width="200"/>

This Blockly block has the following JavaScript definition

```javascript
Blockly.Blocks["publisher_name"] = {
  init: function () {
    this.appendDummyInput("publisher name")
      .setAlign(Blockly.inputs.Align.RIGHT)
      .appendField("publisher name")
      .appendField(new Blockly.FieldTextInput(""), "publisherName");
    this.setNextStatement(true, null); // Can connect to blocks after it
    this.setPreviousStatement(true, null); // Can connect to blocks before it
    this.setTooltip("set publisher name");
    this.setHelpUrl("none");
    this.setColour(120);
  },
};
```

A subsequent python code generation code for this block

```javascript
python.pythonGenerator.forBlock["publisher_name"] = function (block) {
  const code = ``;
  return code;
};
```
