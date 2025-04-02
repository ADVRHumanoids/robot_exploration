# Exploration GUI

The scripts in this python package are based on PyQt5 to create and provide a GUI to interact with the exploration and inspection task.


# Exploration Mission

The GUI has 4 different tabs:
- Exploration Status
- Send Goal
- Objects
- Data Manager

### Exploration Status
Here the user can monitor the execution of the task and abort it with the 'cancel' button.

### Send Goal
In this tab the user can select the object of interest (from a drop down menu or through a text box) and send the task to the robot.
The task can be `Inspect', in which the robot reaches N poses around the object and collects images/data of it. Another possibility is to press 'Reach' to simply send the robot in the object neighborhood.

### Objects
Is used to visualize the data of the objects there were recognized and saved.

### Data Manager
It contains a button to manually save the data of the current objects/areas being visualized by the robot.

## Note
In the case the object specified is not known, the robot will start to explore the environment until the object is found.

In the case of multiple objects of the same class, at the moment, it is considered the first of of that class that was found and stored in the EnvironmentKnowledgeManager.