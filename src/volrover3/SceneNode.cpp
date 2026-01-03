#include <volrover3/SceneNode.h>
#include <cvc/app.h>
#include <vtkProp.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <algorithm>

// Static member for main thread callback
SceneNode::MainThreadCallback SceneNode::s_mainThreadCallback;

void SceneNode::setMainThreadCallback(MainThreadCallback callback)
{
    s_mainThreadCallback = callback;
}

void SceneNode::runOnMainThread(std::function<void()> func)
{
    if (s_mainThreadCallback) {
        s_mainThreadCallback(func);
    } else {
        // No callback set, execute immediately (may not be thread-safe!)
        func();
    }
}

SceneNode::SceneNode(const std::string& statePath)
    : state_object<SceneNode>(statePath)
    , m_visible(true)
    , m_renderer(nullptr)
{
    // Initialize visible state
    if (!statePath.empty()) {
        getState("visible").value(1);  // Default to visible
    }
}

SceneNode::~SceneNode()
{
    // Disconnect from state tree before derived class destructor completes
    // to prevent pure virtual method calls during destruction
    disconnectState();
}

void SceneNode::addToRenderer(vtkRenderer *renderer)
{
    m_renderer = renderer;
    if (m_visible && getProp()) {
        renderer->AddViewProp(getProp());
    }

    for (auto &child : m_children) {
        child->addToRenderer(renderer);
    }
}

void SceneNode::removeFromRenderer(vtkRenderer *renderer)
{
    if (getProp()) {
        renderer->RemoveViewProp(getProp());
    }

    for (auto &child : m_children) {
        child->removeFromRenderer(renderer);
    }

    m_renderer = nullptr;
}

void SceneNode::update()
{
    for (auto &child : m_children) {
        child->update();
    }
}

void SceneNode::setVisible(bool visible)
{
    if (m_visible == visible)
        return;

    m_visible = visible;

    if (m_renderer && getProp()) {
        if (visible) {
            m_renderer->AddViewProp(getProp());
        } else {
            m_renderer->RemoveViewProp(getProp());
        }
    }

    for (auto &child : m_children) {
        child->setVisible(visible);
    }
}

void SceneNode::addChild(std::shared_ptr<SceneNode> child)
{
    m_children.push_back(child);
    if (m_renderer) {
        child->addToRenderer(m_renderer);
    }
}

void SceneNode::removeChild(std::shared_ptr<SceneNode> child)
{
    auto it = std::find(m_children.begin(), m_children.end(), child);
    if (it != m_children.end()) {
        if (m_renderer) {
            (*it)->removeFromRenderer(m_renderer);
        }
        m_children.erase(it);
    }
}

void SceneNode::handleStateChanged(const std::string& childState)
{
    cvcapp.log(2, str(boost::format("SceneNode::handleStateChanged: %s") % childState));
    
    // Marshal state changes to main thread to avoid Qt/VTK threading issues
    runOnMainThread([this, childState]() {
        
        // Handle visible state changes
        if (childState == "visible") {
            int visible = getState("visible").value<int>();
            setVisible(visible != 0);
        }
    });
}
