#ifndef BBOXNODE_H
#define BBOXNODE_H

#include <cvc/volume/bounding_box.h>
#include <vector>
#include <vtkSmartPointer.h>

class vtkActor;
class vtkPolyDataMapper;
class vtkActor2D;
class vtkRenderer;
class vtkMatrix4x4;

// Simple VTK wrapper for bounding box visualization
// Not a SceneNode - controlled by parent GraphicsNode's show_bbox state
class BBoxNode {
public:
  BBoxNode();
  ~BBoxNode();

  void addToRenderer(vtkRenderer *renderer);
  void removeFromRenderer(vtkRenderer *renderer);

  void setBoundingBox(const cvc::bounding_box &bbox);
  cvc::bounding_box getBoundingBox() const { return m_bbox; }

  void setTransform(vtkMatrix4x4 *transform);

  void setColor(double r, double g, double b);
  void getColor(double &r, double &g, double &b) const;
  void setLineWidth(double width);

  // Coordinate label controls
  void setCoordinatesVisible(bool visible);
  bool getCoordinatesVisible() const { return m_coordinatesVisible; }

  void setCoordinateLabelColor(double r, double g, double b);
  void getCoordinateLabelColor(double &r, double &g, double &b) const;

  void setCoordinateLabelFontSize(int size);
  int getCoordinateLabelFontSize() const { return m_coordinateLabelFontSize; }

private:
  void createBBox();
  void createCoordinateLabels();

  vtkSmartPointer<vtkActor> m_actor;
  vtkSmartPointer<vtkPolyDataMapper> m_mapper;
  cvc::bounding_box m_bbox;
  vtkSmartPointer<vtkMatrix4x4> m_transform; // Store world transform for coordinate labels

  // Coordinate label members
  std::vector<vtkSmartPointer<vtkActor2D>> m_coordinateLabelActors;
  bool m_coordinatesVisible;
  double m_coordinateLabelColor[3];
  int m_coordinateLabelFontSize;
  vtkRenderer *m_renderer; // Store renderer to re-add labels when recreated
};

#endif // BBOXNODE_H
