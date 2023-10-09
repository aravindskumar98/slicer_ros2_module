import slicer
import vtk
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
import qt

try:
    import yaml
except:
    pip_install('pyyaml')
    import yaml

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
    customLayoutId = 100  # an arbitrary ID number for the custom layout
    layoutManager.layoutLogic().GetLayoutNode().AddLayoutDescription(customLayoutId, THREE_3D_LAYOUT)

    # Set the application layout to the custom layout
    layoutManager.setLayout(customLayoutId)

    # Extract the 3D widgets for stereo cameras
    threeDWidget1 = layoutManager.threeDWidget(0)
    threeDWidget2 = layoutManager.threeDWidget(1)

    # Create a new window for the stereo cameras
    popupWindow = qt.QWidget()
    popupWindow.setWindowTitle("Stereo Cameras")
    popupWindow.setLayout(qt.QHBoxLayout())
    popupWindow.layout().addWidget(threeDWidget1)
    popupWindow.layout().addWidget(threeDWidget2)
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
