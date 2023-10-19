
import os
import slicer
import vtk
import numpy as np
from slicer.ScriptedLoadableModule import ScriptedLoadableModule, ScriptedLoadableModuleWidget, ScriptedLoadableModuleLogic
from lib.utils import *
try:
    import yaml
except:
    pip_install('pyyaml')
    import yaml


class StereoControlModule(ScriptedLoadableModule):
    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        parent.title = "Stereo Control"
        parent.categories = ["IGT"]
        parent.dependencies = ["ROS2"]
        parent.contributors = ["Aravind Sunil Kumar"]
        parent.helpText = """
        This is a simple module for controlling the stereo cameras using a robotic arm. Primary use case is for medical robotics.
        """
        parent.acknowledgementText = """
        Lorem ipsum dolor sit amet, consectetur adipiscing elit.
        """

class StereoControlModuleLogic(ScriptedLoadableModuleLogic):
    def __init__(self):
        """
        The above function initializes various variables and objects related to the layout manager, 3D
        widgets, view nodes, camera nodes, and transforms.
        """
        self.layoutManager = slicer.app.layoutManager()
        self.threeDWidget1 = self.layoutManager.threeDWidget(0)
        self.threeDWidget2 = self.layoutManager.threeDWidget(1)
        # Get the view nodes from the 3D widgets
        self.viewNode1 = self.threeDWidget1.mrmlViewNode()
        self.viewNode2 = self.threeDWidget2.mrmlViewNode()
        # Get the active camera nodes for both views
        self.cameraNode1 = slicer.modules.cameras.logic(
            ).GetViewActiveCameraNode(self.viewNode1)
        self.cameraNode2 = slicer.modules.cameras.logic(
            ).GetViewActiveCameraNode(self.viewNode2)
        
        self.stereoCamera = StereoCamera(self.cameraNode1, self.cameraNode2)
        # These are the transforms that are used to store state of the controller arm
        self.currentControllerTransform = vtk.vtkMatrix4x4()
        self.nextControllerTransform = vtk.vtkMatrix4x4()


    def setBaseline(self, baseline):
        self.stereoCamera.SetBaseline(baseline)

          
    def extractPositionFromTransform(self, cameraTransform):
        return [cameraTransform.GetElement(i, 3) for i in range(3)]

    def multiplyMatrices(self, mat1, mat2):
        result = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Multiply4x4(mat1, mat2, result)
        return result


    def displaceCameraV2(self, displacementRotationMatrix4x4, positionDisplacementVector): #TODO: position, matrix also make it 3x3 matrix
        
        currentCentralCameraPosition, centralCameraCurrentTransform = self.stereoCamera.GetCentralCameraPositionAndTransform()

        positionDisplacementMatrix = vtk.vtkMatrix4x4() #Here, the positionDisplacementVector is changed to a matrix
        for i in range(3):
            centralCameraCurrentTransform.SetElement(i, 3, 0)
            positionDisplacementMatrix.SetElement(i, 3, positionDisplacementVector[i]) 

        # Calculate the new position displacement matrix by multiplying the central camera's current transform with the position displacement matrix
        newPositionDisplacementMatrix = self.multiplyMatrices(centralCameraCurrentTransform, positionDisplacementMatrix)
        # Calculate the new central camera transform by multiplying the central camera's current transform with the displacement rotation matrix
        centralCameraNextTransform = self.multiplyMatrices(centralCameraCurrentTransform, displacementRotationMatrix4x4)

        # Extract the position displacement vector from the new position displacement matrix
        positionDisplacementVector = self.extractPositionFromTransform(newPositionDisplacementMatrix)

        # Add the position displacement vector to the current central camera position to get the new central camera position
        for i, pos in enumerate(positionDisplacementVector):
            centralCameraNextTransform.SetElement(i, 3, pos + currentCentralCameraPosition[i])

        self.stereoCamera.SetCentralCameraTransform(centralCameraNextTransform)

    def createSittingTransforms(self, transformName):
        if not slicer.mrmlScene.GetFirstNodeByName(transformName):
            sittingTransform = slicer.vtkMRMLLinearTransformNode()
            sittingTransform.SetName(transformName)
            slicer.mrmlScene.AddNode(sittingTransform)
        return slicer.mrmlScene.GetFirstNodeByName(transformName)


    def setup(self):
        """
        The setup function creates and initializes various nodes and variables for a ROS2 node in Slicer,
        including a TF2 lookup node, a subscriber node, and camera transform nodes.
        """
        ros2Node = slicer.mrmlScene.GetFirstNodeByName('ros2:node:slicer')
        self.lookupNode = ros2Node.CreateAndAddTf2LookupNode(
            "MTMR_base", "MTMR")
        self.lookupNodeID = self.lookupNode.GetID()
        self.scale_factor = 4

        # Create a subscriber node to receive messages from the ROS2 topic
        # this receives the button state from the controller
        self.buttonSubscriber = ros2Node.CreateAndAddSubscriberNode(
            "vtkMRMLROS2SubscriberJoyNode", "/console/camera")
        
        # Sitting transforms are used to visualize the camera positions in the 3D view
        if not slicer.mrmlScene.GetFirstNodeByName("leftCameraSittingTransform"):
            self.leftCameraSittingTransform = slicer.vtkMRMLLinearTransformNode()
            self.leftCameraSittingTransform.SetName("leftCameraSittingTransform")
            slicer.mrmlScene.AddNode(self.leftCameraSittingTransform)

        if not slicer.mrmlScene.GetFirstNodeByName("rightCameraSittingTransform"):
            self.rightCameraSittingTransform = slicer.vtkMRMLLinearTransformNode()
            self.rightCameraSittingTransform.SetName("rightCameraSittingTransform")
            slicer.mrmlScene.AddNode(self.rightCameraSittingTransform)

        self.moveCamera = False
        self.startFlag = False

        # Add observers to the subscriber node and the lookup node
        # The callback function is called whenever the node receives a message/event
        self.buttonObserverID = self.buttonSubscriber.AddObserver(
            "ModifiedEvent", self._buttonCallback)
        observerId = self.lookupNode.AddObserver(
            slicer.vtkMRMLTransformNode.TransformModifiedEvent, self._callback)
        
    def setupWithoutHardware(self):
        # Sitting transforms are used to visualize the camera positions in the 3D view
        self.scale_factor = 4

        if not slicer.mrmlScene.GetFirstNodeByName("leftCameraSittingTransform"):
            self.leftCameraSittingTransform = slicer.vtkMRMLLinearTransformNode()
            self.leftCameraSittingTransform.SetName("leftCameraSittingTransform")
            slicer.mrmlScene.AddNode(self.leftCameraSittingTransform)

        if not slicer.mrmlScene.GetFirstNodeByName("rightCameraSittingTransform"):
            self.rightCameraSittingTransform = slicer.vtkMRMLLinearTransformNode()
            self.rightCameraSittingTransform.SetName("rightCameraSittingTransform")
            slicer.mrmlScene.AddNode(self.rightCameraSittingTransform)

        self.stereoCamera.addSittingTransforms(self.leftCameraSittingTransform, self.rightCameraSittingTransform)

        self.moveCamera = False
        self.startFlag = False

        transformNode = slicer.mrmlScene.GetFirstNodeByName('LinearTransform')
        vtkTransform = transformNode.GetTransformToParent()
        observerTag = vtkTransform.AddObserver(vtk.vtkCommand.ModifiedEvent, self.onTransformModified)

        self.currentControllerTransform = vtk.vtkMatrix4x4()
        self.currentControllerTransform.DeepCopy(vtkTransform.GetMatrix())

    def onTransformModified(self, caller, event):
        print("Transform modified!")
        caller.GetMatrix(self.nextControllerTransform)
        displacement, positionDisplacementVector = findDisplacementTransform(
            self.currentControllerTransform, self.nextControllerTransform, self.scale_factor)
        self.displaceCameraV2(displacement, positionDisplacementVector)
        # copy contents of nextControllerTransform into currentControllerTransform they are vtkMatrix4x4 objects
        self.currentControllerTransform.DeepCopy(self.nextControllerTransform)


    def SetAbsoluteCameraPosition(self, xDisp = 0.0, yDisp = 0.0, zDisp = 0.0):
        self.stereoCamera.SetAbsoluteCameraPosition(xDisp, yDisp, zDisp)

    def _buttonCallback(self, caller, event):
        """
        The function checks the value of a button and performs different actions based on
        its value.
        
        :param caller: The `caller` parameter is the object that triggered the callback function. 
        :param event: The "event" parameter is used to capture the event
        that triggered the callback. 
        """
        msg_yaml = caller.GetLastMessageYAML()
        msg = yaml.load(msg_yaml, Loader=yaml.FullLoader)
        button = msg['buttons'][0]

        if button == 1 and self.moveCamera == False:
            self.startCameraControl()
        elif button == 0 and self.moveCamera == True:
            self.stopCameraControl()
        else:
            print(f"Received button value: {button}")

    def helperLogCameraStartState(self):
        self.leftCameraInitialTransform, self.focal_disp_magnitude_left_initial = self.GetCameraTransform(self.cameraNode1)
        self.leftCameraInitialTransform = vtk.vtkMatrix4x4()

    def startCameraControl(self):
        self.currentControllerTransform = vtk.vtkMatrix4x4()
        # self.currentControllerTransform.DeepCopy(
        #     self.lookupNode.GetMatrixTransformToParent())
        self.lookupNode.GetMatrixTransformToParent(self.currentControllerTransform)
        self.moveCamera = True
        if self.startFlag:
            self.helperLogCameraStartState()
            self.startFlag = False

    def stopCameraControl(self):
        self.moveCamera = False

    def _callback(self, caller, event):
        """
        The `_callback` function updates the camera position based on the movement of a controller.
        
        :param caller: The "caller" parameter is an object that is calling the _callback method. 
        :param event: The "event" parameter is an object that represents the event that triggered the
        callback function. 
        """
        if self.moveCamera:
            caller.GetMatrixTransformToParent(self.nextControllerTransform)
            displacement, positionDisplacementVector = findDisplacementTransform(
                self.currentControllerTransform, self.nextControllerTransform, self.scale_factor)
            self.displaceCamera(displacement, positionDisplacementVector)
            # copy contents of nextControllerTransform into currentControllerTransform they are vtkMatrix4x4 objects
            self.currentControllerTransform.DeepCopy(self.nextControllerTransform)

    
    def createStereoLayout(self):
        # Move your stereo layout creation code here
        print("Logic called!")


class StereoControlModuleWidget(ScriptedLoadableModuleWidget):
    def setup(self):
        ScriptedLoadableModuleWidget.setup(self)
        
        # Load the UI file
        uiFilePath = os.path.join(os.path.dirname(__file__), 'Resources/UI/StereoControlModule.ui')
        self.ui = slicer.util.loadUI(uiFilePath)
        self.layout.addWidget(self.ui)
        self.window = None
        
        # Connect the button to a function
        self.ui.demoButton.clicked.connect(self.onDemoButtonClick)

    def onDemoButtonClick(self):
        """
        The function `onDemoButtonClick` creates a custom layout window, sets up a stereo control module
        logic, defines an offset, resets the camera position, and displaces the camera.
        """     
        if self.window:
            self.window.close()

        # Set position and resolution of the window
        self.window = createCustomLayout([100,100],[1280*2,720])
        self.window.show()

        cylinderModel, cylinderTransform = createInteractableCylinder()
        # cylinderModel.GetDisplayNode().SetVisibility(1)

        # Create a stereo control module logic object
        # test_stereo_camera()
        print("Button clicked!")

        self.logic = StereoControlModuleLogic()

        self.logic.setBaseline(20)
        self.logic.SetAbsoluteCameraPosition(0.0,-200.0,0.0) #TODO: add VTK matrix as a parameter too

        self.logic.setupWithoutHardware()
        # self.logic.defineOffset(-5, 0, 0)
        # self.logic.ResetCameraPosition(0,-200,0)
        
        self.logic.displaceCameraV2(vtk.vtkMatrix4x4(), [0,0,0])
        self.logic.displaceCameraV2(vtk.vtkMatrix4x4(), [0,0,0])


        
        





