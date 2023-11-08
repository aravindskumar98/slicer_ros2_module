### Requirements for ROS2 Services in MRML Nodes

3. **Asynchronous Handling**: The service should handle requests asynchronously, allowing the 3D Slicer application to remain responsive while the service is processing.

4. **Error Handling**: Robust error handling must be built into the service to manage timeouts, connection errors, and invalid responses.

5. **Data Type Support**: The service must support all data types required by the application, including primitives, arrays, and complex types defined by ROS2 messages.

6. **User-defined Callbacks**: Users should be able to define callback functions that will handle service responses.

### Questions

1. Keep connection alive? Check that the server is alive. Before sending out a request.
2. We assume that the answer will be provided as a callback/events. It is for the users to attach a callback to the event. (This would be adding an observer?)
3. If there is already a request pending, respond with a false.
4. instead of spin future thing, do a wait or something.
5. SendRequest function needs to be created.
6. Replace Subscriber with Service and figrue out what stays and what does not.















### Potential Signature for ROS2 Services in MRML 
The signature for creating and using a ROS2 service in the MRML context would likely mirror the pattern established by publishers and subscribers. Here's a potential C++ class method signature for creating a service:
```cpp
// Method to create and add a service node
vtkMRMLROS2ServiceNode* CreateAndAddServiceNode(const std::string& serviceType, const std::string& serviceName);
```

This method would create a service node of the specified type and service name.

### Calling a ROS2 Service

To call a service, you might have a method like this:

```cpp
// Method to call a service with the given request and get a response
template<typename ServiceRequestType, typename ServiceResponseType>
bool CallService(const std::string& serviceName, const ServiceRequestType& request, ServiceResponseType& response);
```

The `CallService` method would be a templated method allowing for different request and response types.

Potential Design

1. **Service Node Class**: Create a `vtkMRMLROS2ServiceNode` class to represent the service in the MRML framework. This class would store the service information and handle sending requests and receiving responses.

2. **Service Manager**: Implement a `vtkMRMLROS2ServiceManager` class to manage all service nodes, dispatching service calls, and handling callbacks.

3. **Interface Methods**: Define methods in `vtkMRMLROS2Node` to create, call, and delete service nodes.

4. **Callback Handling**: Each service node could maintain a list of callback functions registered by the user, which are triggered upon receiving a response.

5. **Serialization/Deserialization**: Integrate utilities for serializing and deserializing ROS2 message types to be sent over the network.

6. **Thread Management**: Utilize a thread pool or an asynchronous I/O framework to manage service call concurrency.

7. **ROS2 Node Handle**: Each service node would have a handle to the underlying ROS2 node to interface with the ROS2 graph.

Example of Service Creation and Call

```cpp
// Example of creating a service node and making a service call
vtkMRMLROS2Node* rosNode = slicer.util.getModuleLogic('ROS2').GetDefaultROS2Node();
vtkMRMLROS2ServiceNode* myService = rosNode->CreateAndAddServiceNode('MyServiceType', 'my_service');

MyServiceRequestType request;
MyServiceResponseType response;

if (rosNode->CallService('my_service', request, response)) {
  // Handle response
}
```

This is a high-level overview and would need to be fleshed out with details specific to the data types and services used within your application. It's also important to consider threading and the asynchronous nature of network services when integrating this into a UI-driven application like 3D Slicer.

