#ifndef __vtkMRMLROS2ParameterNode_h
#define __vtkMRMLROS2ParameterNode_h

// MRML includes
#include <vtkMRMLNode.h>
#include <vtkCommand.h>
#include <vtkSlicerROS2ModuleMRMLExport.h>
#include <deque>
// #include <utility>

// forward declaration for internals
class vtkMRMLROS2ParameterInternals;

class VTK_SLICER_ROS2_MODULE_MRML_EXPORT vtkMRMLROS2ParameterNode : public vtkMRMLNode {
    // friend declarations
    friend class vtkMRMLROS2ParameterInternals;
    friend class vtkMRMLROS2NodeNode;

   protected:
    vtkMRMLROS2ParameterNode(void);
    ~vtkMRMLROS2ParameterNode(void);

   public:

    enum Events
    {
      ParameterModifiedEvent = vtkCommand::UserEvent + 54
    };

    vtkTypeMacro(vtkMRMLROS2ParameterNode, vtkMRMLNode);
    typedef vtkMRMLROS2ParameterNode SelfType;
    static SelfType* New(void);
    void PrintSelf(ostream& os, vtkIndent indent) override;
    vtkMRMLNode* CreateNodeInstance(void) override;
    const char* GetNodeTagName(void) override;

    typedef std::pair<std::string, std::string> ParameterKey;  // pair: {nodeName, parameterName} todo: is it used
    bool AddToROS2Node(const char* nodeId, const std::string& monitoredNodeName);

    /*! Get the name of the node holding the parameters we're looking for. */
    inline const std::string & GetNodeName(void) const { // todo:: MonitoredNode
      return mMonitoredNodeName;
    }

    bool SetupParameterEventSubscriber(void); // todo: protected
    bool IsAddedToROS2Node(void) const;
    bool IsParameterServerReady(void) const; // todo: IsMonitoredNodeReady

    /* Add a node and parameter to monitor */
    bool AddParameter(const std::string& parameterName); // todo: AddParameter
    /* Remove a parameter that is being monitored. If no parameters are being monitored for a node, stop monitoring the node as well*/
    bool RemoveParameter(const std::string& parameterName); // todo: RemoveParameter

    bool IsParameterValueSet(const std::string& parameterName) const; // IsParameterSet

    /* Main methods, recommended for C++ users since we can check return code and avoid copy for result.
     Returns data type if the parameter is monitored. Else it returns an empty string */
    std::string GetParameterType(const std::string& parameterName, std::string& result); // todo: bool return, false is not yet known
    /* convenience methods for users to skip pair creation, mostly for Python users.  Returns an empty string if the parameter type is not yet known. */
    inline std::string GetParameterType(const std::string& parameterName) {
        std::string result;
        // TODO : Add Warning
        GetParameterType(parameterName, result);
        return result;
    }

    /*! Main methods, recommended for C++ users since we can check return code and avoid copy for result.
     Prints value of a monitored parameter after converting it to a string */
    bool PrintParameterValue(const std::string& parameterName, std::string& result); // todo: remove Value in method name, return false is parameter not yet set (update documention)
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline std::string PrintParameterValue(const std::string& parameterName) {
        std::string result;
        PrintParameterValue(parameterName, result);
        return result;
    }

// todo: in RbotoNode, make sure we check with IsParameterSet and GetParameterType 

    /* Main methods, recommended for C++ users since we can check return code and avoid copy for result.
    Returns true if it is a boolean and it is set. Users should always make sure that the key exists
    and the parameter type is boolean before calling this method*/
    bool GetParameterAsBool(const std::string& parameterName, bool& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline bool GetParameterAsBool(const std::string& parameterName) {
        bool result;
        GetParameterAsBool(parameterName, result);
        return result; // todo : try and return a tuple
    }
    // TODO: Exception vs Tuple - keep a note for subscriber

    /* Returns the value of the parameter if it is an Integer . Users should always make sure the key exists and
    the parameter type is an integer with GetParameterType before calling this method.
    Main methods, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsInteger(const std::string& parameterName, int& result);
    /* convenience methods for users to return output, mostly for Python users */
    int GetParameterAsInteger(const std::string& parameterName);



    /* Returns the value of the parameter if it is a double. Users should always make sure the key exists and
    the parameter type is a double with GetParameterType before calling this method.

    Main methods, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsDouble(const std::string& parameterName, double& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline double GetParameterAsDouble(const std::string& parameterName) {
        double result;
        GetParameterAsDouble(parameterName, result);
        return result;
    }

    /* Returns the value of the parameter if it is a string. Users should always make sure that the key exists
    and the parameter type is string before calling this method

    Main methods, recommended for C++ users since we can check return code and avoid copy for result.*/
    bool GetParameterAsString(const std::string& parameterName, std::string& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline std::string GetParameterAsString(const std::string& parameterName) {
        std::string result;
        GetParameterAsString(parameterName, result);
        return result;
    }

    /* Returns the value of the parameter if it is a vector of bools. Users should always make sure the key exists and
    the parameter type is a vector of bools with GetParameterType before calling this method.

    Main methods, recommended for C++ users since we can check return code and avoid copy for result. */
    // bool GetParameterAsVectorOfBools(const std::string & parameterName, std::vector<bool> & result);
    // /* convenience methods for users to skip pair creation, mostly for Python users */

    // TODO : unable to build for some reasong
    // inline std::vector<bool> GetParameterAsVectorOfBools(const std::string &parameterName) {
    //   std::vector<bool> result;
    //   // GetParameterAsVectorOfBools(parameterName, result);
    //   return result;
    // }

    /* Returns the value of the parameter if it is a vector of ints. Users should always make sure the key exists and
    the parameter type is a vector of ints with GetParameterType before calling this method.

    Main methods, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsVectorOfIntegers(const std::string& parameterName, std::vector<int64_t>& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline std::vector<int64_t> GetParameterAsVectorOfIntegers(const std::string& parameterName) {
        std::vector<int64_t> result;
        GetParameterAsVectorOfIntegers(parameterName, result);
        return result;
    }

    /* Returns the value of the parameter if it is a vector of doubles. Users should always make sure the key exists and
    the parameter type is a vector of doubles with GetParameterType before calling this method.

    Main methods, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsVectorOfDoubles(const std::string& parameterName, std::vector<double>& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline std::vector<double> GetParameterAsVectorOfDoubles(const std::string& parameterName) {
        std::vector<double> result;
        GetParameterAsVectorOfDoubles(parameterName, result);
        return result;
    }

    /* Returns the value of the parameter if it is a vector of strings. Users should always make sure the key exists and
    the parameter type is a vector of strings with GetParameterType before calling this method.

    Main methods, recommended for C++ users since we can check return code and avoid copy for result. */
    bool GetParameterAsVectorOfStrings(const std::string& parameterName, std::vector<std::string>& result);
    /* convenience methods for users to skip pair creation, mostly for Python users */
    inline std::vector<std::string> GetParameterAsVectorOfStrings(const std::string& parameterName) {
        std::vector<std::string> result;
        GetParameterAsVectorOfStrings(parameterName, result);
        return result;
    }

    virtual void ParameterSet(void)
    {
      this->InvokeCustomModifiedEvent(vtkMRMLROS2ParameterNode::ParameterModifiedEvent);
    }

    // Save and load
    virtual void ReadXMLAttributes(const char** atts) override;
    virtual void WriteXML(std::ostream& of, int indent) override;
    void UpdateScene(vtkMRMLScene* scene) override;

   protected:

    bool CheckParameterExistsAndIsSet(const std::string &parameterName) const;

   // vector to store all parameter names that are monitored by the node. This is used for saving and reloading state.
    std::vector<std::string> MonitoredParameterNamesCache = {};
    void SetMonitoredParameterNamesCache(const std::vector<std::string>& MonitoredParameterNamesCache); // todo: line below and below - make private + use the word cache/temp in there + remove m 
    std::vector<std::string> GetMonitoredParameterNamesCache();


   protected:
    vtkMRMLROS2ParameterInternals* mInternals = nullptr; // todo: should we use make_unique?
    std::string mMRMLNodeName = "ros2:param:undefined";
    std::string mMonitoredNodeName = "undefined";
    bool mIsInitialized = false;

    // For ReadXMLAttributes
    vtkGetMacro(mMRMLNodeName, std::string);
    vtkSetMacro(mMRMLNodeName, std::string);
    vtkGetMacro(mMonitoredNodeName, std::string);
    vtkSetMacro(mMonitoredNodeName, std::string);

};

#endif  // __vtkMRMLROS2ParameterNode_h

// ros2 = slicer.mrmlScene.GetFirstNodeByName('ros2:node:test_node')
