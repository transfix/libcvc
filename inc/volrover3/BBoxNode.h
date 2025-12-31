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
    
    // Coordinate label controls
    void setCoordinatesVisible(bool visible);
    bool coordinatesVisible() const { return m_coordinatesVisible; }
    
    void setCoordinateLabelColor(double r, double g, double b);
    void getCoordinateLabelColor(double& r, double& g, double& b) const;
    
    void setCoordinateLabelFontSize(int size);
    int coordinateLabelFontSize() const { return m_coordinateLabelFontSize; }

private:
    void createBBox();
    void createCoordinateLabels();

    vtkSmartPointer<vtkActor> m_actor;
    vtkSmartPointer<vtkPolyDataMapper> m_mapper;
    cvc::bounding_box m_bbox;
    
    // Coordinate label members
    std::vector<vtkSmartPointer<vtkActor2D>> m_coordinateLabelActors;
    bool m_coordinatesVisible;
    double m_coordinateLabelColor[3];
    int m_coordinateLabelFontSize;
    vtkRenderer* m_renderer;  // Store renderer to re-add labels when recreated
};

#endif // BBOXNODE_H
