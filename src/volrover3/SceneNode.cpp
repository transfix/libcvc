#include <volrover3/SceneNode.h>
#include <vtkProp.h>
#include <vtkRenderer.h>
#include <algorithm>

SceneNode::SceneNode()
    : m_visible(true)
    , m_renderer(nullptr)
{
}

SceneNode::~SceneNode()
{
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
