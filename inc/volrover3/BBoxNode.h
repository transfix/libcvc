#ifndef BBOXNODE_H
#define BBOXNODE_H

#include <volrover3/SceneNode.h>
#include <cvc/bounding_box.h>
#include <vtkSmartPointer.h>
#include <vector>

class vtkActor;
class vtkPolyDataMapper;
class vtkActor2D;
class vtkRenderer;

class BBoxNode : public SceneNode
{
public:
    BBoxNode();
    ~BBoxNode() override;

    vtkProp* getProp() override;
    void addToRenderer(vtkRenderer* renderer) override;
    void removeFromRenderer(vtkRenderer* renderer) override;
    
    void setBoundingBox(const cvc::bounding_box& bbox);
    void setColor(double r, double g, double b);
    void setLineWidth(double width);
    
    // Tick label controls
    void setTicksVisible(bool visible);
    bool getTicksVisible() const { return m_ticksVisible; }
    
    void setTickInterval(double interval);
    double getTickInterval() const { return m_tickInterval; }
    
    void setTickLabelColor(double r, double g, double b);
    void getTickLabelColor(double& r, double& g, double& b) const;
    
    void setTickLabelFontSize(int size);
    int getTickLabelFontSize() const { return m_tickLabelFontSize; }

private:
    void createBBox();
    void createTickLabels();

    vtkSmartPointer<vtkActor> m_actor;
    vtkSmartPointer<vtkPolyDataMapper> m_mapper;
    cvc::bounding_box m_bbox;
    
    // Tick label members
    std::vector<vtkSmartPointer<vtkActor2D>> m_tickLabelActors;
    bool m_ticksVisible;
    double m_tickInterval;
    double m_tickLabelColor[3];
    int m_tickLabelFontSize;
};

#endif // BBOXNODE_H
