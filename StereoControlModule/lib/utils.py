import slicer
import vtk
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
import qt


def helper_print_matrix(matrix):
    for i in range(4):
        for j in range(4):
            print(f"{matrix.GetElement(i,j):.2f}", end = " ")
        print()

class MonoCamera:

    def __init__(self, cameraMRMLNode, description=""):
        # TODO verify that cameraObject is a vtkMRMLCameraNode

        self.cameraNode = cameraMRMLNode
        self.description = description

    def addSittingTransform(self, sittingTransform):
        self.sittingTransform = sittingTransform

    def SetBaselineOffset(self, baselineOffset):
        self.baselineOffset = baselineOffset
        self.matrixOffsetCamera = vtk.vtkMatrix4x4()

        self.matrixOffsetCamera.SetElement(0, 3, baselineOffset / 2)
        self.matrixOffsetCamera.SetElement(1, 3, 0)
        self.matrixOffsetCamera.SetElement(2, 3, 0)

    def extractPositionFromTransformMatrix(self, transformMatrix):
        return [transformMatrix.GetElement(i, 3) for i in range(3)]
    

    def GetCameraTransform(self):
        """
        The function `GetCameraTransform` calculates the camera transform matrix and magnitude based on the
        position, focal point, and view up vectors of a camera node. This is required as 3D Slicer does not
        provide a function to get the camera transform matrix directly. 

        Z is View Up
        Y is Focal Direction
        X is Left to Right
        
        :param cameraNode: The `cameraNode` parameter is an object representing a camera in a 3D scene. It
        contains information about the camera's position, focal point, and view up vector
        :return: the camera transform matrix and the magnitude of the focal displacement.
        """
        position = self.cameraNode.GetPosition()
        focalPoint = self.cameraNode.GetFocalPoint()
        viewUp = self.cameraNode.GetViewUp()

        focalDisplacement = np.array(focalPoint) - np.array(position)
        self.magnitude = np.linalg.norm(focalDisplacement)

        zz = viewUp
        zz /= np.linalg.norm(zz)  # Normalize

        yy = focalDisplacement
        yy /= np.linalg.norm(yy)  # Normalize

        xx = np.cross(yy, zz)
        xx /= np.linalg.norm(xx)  # Normalize

        self.cameraMatrix = vtk.vtkMatrix4x4()
        self.cameraMatrix.Identity()
        for i in range(3):
            self.cameraMatrix.SetElement(i, 0, xx[i])
            self.cameraMatrix.SetElement(i, 1, yy[i])
            self.cameraMatrix.SetElement(i, 2, zz[i])
            self.cameraMatrix.SetElement(i, 3, position[i])

        self.cameraPosition = self.extractPositionFromTransformMatrix(self.cameraMatrix)

        self.cameraRotation4x4 = vtk.vtkMatrix4x4()
        # self.cameraMatrix.DeepCopy(self.cameraRotation4x4)
        # for i in range(3):
        #     self.cameraRotation4x4.SetElement(i, 3, 0)

        return self.cameraMatrix, self.magnitude, self.cameraPosition, self.cameraRotation4x4
    
    def SetCameraTransform(self, centralCameraMatrix, magnitude):

        cameraMatrix = vtk.vtkMatrix4x4()
        vtk.vtkMatrix4x4.Multiply4x4(
            centralCameraMatrix, self.matrixOffsetCamera, cameraMatrix)
        
        position = [cameraMatrix.GetElement(i, 3) for i in range(3)]

        yy = [cameraMatrix.GetElement(i, 1) for i in range(3)]
        focalDisplacement = np.array(yy) * magnitude
        focalPoint = np.array(position) + focalDisplacement

        viewUp = [cameraMatrix.GetElement(i, 2) for i in range(3)]

        self.cameraNode.SetPosition(position)
        self.cameraNode.SetFocalPoint(focalPoint)
        self.cameraNode.SetViewUp(viewUp)

        # Apply the sitting transform
        self.sittingTransform.SetMatrixTransformToParent(cameraMatrix)

    def SetAbsoluteCameraPosition(self, xDisp, yDisp, zDisp):
        self.cameraNode.SetPosition(xDisp + self.baselineOffset/2 , yDisp, zDisp)
        self.cameraNode.SetFocalPoint(0, 0, 0)
        self.cameraNode.SetViewUp(0, 0, 1)

class StereoCamera:
    
    def __init__(self, cameraLeft, cameraRight):
        
        # check if camera1 and camera2 are MonoCamera objects
        if isinstance(cameraLeft, MonoCamera) and isinstance(cameraRight, MonoCamera):
            self.cameraLeft = cameraLeft
            self.cameraRight = cameraRight
        else:
            self.cameraLeft = MonoCamera(cameraLeft)
            self.cameraRight = MonoCamera(cameraRight)
        print("StereoCamera: init")

    def addSittingTransforms(self, sittingTransformLeft, sittingTransformRight):
        self.cameraLeft.addSittingTransform(sittingTransformLeft)
        self.cameraRight.addSittingTransform(sittingTransformRight)

    def SetBaseline(self, baseline):
        leftOffset = -1 * baseline / 2
        rightOffset = baseline / 2

        self.cameraLeft.SetBaselineOffset(leftOffset)
        self.cameraRight.SetBaselineOffset(rightOffset)

    def GetCentralCameraPositionAndTransform(self):
        # This represents the effective central camera which is the middle of the two cameras
        matrixLeft, magnitudeLeft, positionLeft, rotationLeft = self.cameraLeft.GetCameraTransform()
        matrixRight, magnitudeRight, positionRight, rotationRight = self.cameraRight.GetCameraTransform()

        # Calculate the central camera matrix
        matrixCentral = vtk.vtkMatrix4x4()
        matrixCentral.DeepCopy(matrixLeft)

        # Calculate the central camera position
        positionCentral = [(positionLeft[i] + positionRight[i]) / 2 for i in range(3)]

        # remember the magnitudes for later
        self.magnitudeLeft = magnitudeLeft
        self.magnitudeRight = magnitudeRight

        return positionCentral, matrixCentral
    
    def SetCentralCameraTransform(self, matrixCentral):
        # Set the central camera position
        self.cameraLeft.SetCameraTransform(matrixCentral, self.magnitudeLeft)
        self.cameraRight.SetCameraTransform(matrixCentral, self.magnitudeRight)

    def SetAbsoluteCameraPosition(self, xDisp, yDisp, zDisp):
        # Set the central camera position
        self.cameraLeft.SetAbsoluteCameraPosition(xDisp, yDisp, zDisp)
        self.cameraRight.SetAbsoluteCameraPosition(xDisp, yDisp, zDisp)



def test_stereo_camera():
    camera = StereoCamera(MonoCamera(), MonoCamera())

def createCustomLayout(position, size):
    """
    The function `createCustomLayout` creates a custom layout in the Slicer application with three 3D
    views and displays them in a popup window.
    
    :param position: The position parameter is the position of the top-left corner of the custom layout
    window on the screen. It is a tuple of two integers representing the x and y coordinates,
    respectively. For example, (100, 100) would position the window at coordinates (100, 100) on the
    screen
    :param size: The size parameter is the desired size of the popup window that will display the stereo
    cameras. It is a tuple of two integers representing the width and height of the window
    :return: a `QWidget` object, which represents a window with a layout containing two 3D widgets for
    stereo cameras.
    """

    THREE_3D_LAYOUT = """
        <layout type="vertical" >
            <item>
                <view class="vtkMRMLViewNode" singletontag="1"/>
            </item>
            <item>
                <view class="vtkMRMLViewNode" singletontag="2"/>
            </item>
            <item>
                <view class="vtkMRMLViewNode" singletontag="3"/>
            </item>
        </layout>
        """
    # Register the custom layout with the application layout manager
    layoutManager = slicer.app.layoutManager()
    customLayoutId = 100  # an arbitrary ID number for the custom layout # TODO: make it max plus one, not arbitrary hundred
    layoutManager.layoutLogic().GetLayoutNode().AddLayoutDescription(customLayoutId, THREE_3D_LAYOUT)

    # Set the application layout to the custom layout
    layoutManager.setLayout(customLayoutId)

    # Extract the 3D widgets for stereo cameras
    threeDWidgetLeft = layoutManager.threeDWidget(0) # TODO: retrieve them by name or by some other tag 
    threeDWidgetRight = layoutManager.threeDWidget(1)

    # Create a new window for the stereo cameras
    popupWindow = qt.QWidget()
    popupWindow.setWindowTitle("Stereo Cameras")
    popupWindow.setLayout(qt.QHBoxLayout())
    popupWindow.layout().addWidget(threeDWidgetLeft)
    popupWindow.layout().addWidget(threeDWidgetRight)
    popupWindow.show()

    popupWindow.move(100, 100)
    popupWindow.resize(1280*2, 720)

    return popupWindow


def findDisplacementTransform(startTransform, endTransform, translationScaleFactor = 0.3, rotationScaleFactor = 0.4):
    displacementTransform = vtk.vtkMatrix4x4()

    # end = displacement * start
    # displacement = end * start^-1
    startTransformInverse = vtk.vtkMatrix4x4()
    vtk.vtkMatrix4x4.Invert(startTransform, startTransformInverse)
    vtk.vtkMatrix4x4.Multiply4x4(
        endTransform, startTransformInverse, displacementTransform)

    positionDisplacementVector = [(endTransform.GetElement(
        i, 3) - startTransform.GetElement(i, 3)) * translationScaleFactor for i in range(3)]

    # divide position by scale factor
    displacementTransform.SetElement(
        0, 3, 0)
    displacementTransform.SetElement(
        1, 3, 0)
    displacementTransform.SetElement(
        2, 3, 0)
    
    # displacementTransform = slerp_vtk_rotation_matrix(displacementTransform)
    displacementTransform = interpolateVTKMatrix(displacementTransform, rotationScaleFactor)

    return displacementTransform, positionDisplacementVector

def slerpScipy(r0, r1, t):
    # Times sequence for the start and end rotations
    times = [0, 1]
    combinedRotations = R.concatenate([r0, r1])
    # Create the Slerp object
    slerpInterpolator = Slerp(times, combinedRotations)
    # Obtain the interpolated rotation
    slerpRotation = slerpInterpolator([t])[0]

    # Return the quaternion of the interpolated rotation
    return slerpRotation

def interpolateVTKMatrix(vtkMatrix, t):
    # Convert vtkMatrix4x4 to numpy array
    matrixArray = np.array([[vtkMatrix.GetElement(i, j) for j in range(4)] for i in range(4)])

    # Extract 3x3 rotation matrix
    rMat = matrixArray[0:3, 0:3]
    r1 = R.from_matrix(rMat)

    # Perform slerp between identity quaternion and rotation
    r0 = R.from_euler('xyz', [0, 0, 0])  # Create an identity rotation

    rScaled = slerpScipy(r0, r1, t)
    rTransformed = rScaled.as_matrix()

    # Construct the resulting vtkMatrix4x4
    resultVTKMatrix = vtk.vtkMatrix4x4()
    for i in range(3):
        for j in range(3):
            resultVTKMatrix.SetElement(i, j, rTransformed[i, j])

    return resultVTKMatrix

def createInteractableCylinder():
    cylinderSource = vtk.vtkCylinderSource()
    cylinderSource.SetHeight(20)        # set height as per requirement
    cylinderSource.SetRadius(10)        # set radius as per requirement
    cylinderSource.SetResolution(100)   # set resolution as per requirement
    cylinderSource.Update()

    cylinderModel = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'CylinderModel')
    cylinderModel.SetAndObservePolyData(cylinderSource.GetOutput())

    # Step 2: Create a Transform
    transform = slicer.vtkMRMLLinearTransformNode()
    slicer.mrmlScene.AddNode(transform)

    matrix = vtk.vtkMatrix4x4()
    # Modify the matrix values as per your requirement. This is just an example.
    matrix.Identity()  # start with an identity matrix
    matrix.SetElement(0, 3, 100)  # translation along X
    matrix.SetElement(1, 3, 100)  # translation along Y
    matrix.SetElement(2, 3, 100)   # translation along Z

    transform.SetMatrixTransformToParent(matrix)

    # Step 3: Apply the Transform to the Cylinder Model
    cylinderModel.SetAndObserveTransformNodeID(transform.GetID())    

    return cylinderModel, transform
