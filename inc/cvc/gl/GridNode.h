#ifndef GRIDNODE_H
#define GRIDNODE_H

#include <cvc/gl/GraphicsNode.h>
#include <cvc/volume/bounding_box.h>
#include <vector>
#include <vtkSmartPointer.h>

class vtkActor;
class vtkPolyDataMapper;
class vtkRenderer;
class vtkActor2D;
class vtkTextMapper;

namespace cvc {
namespace gl {

class GridNode : public GraphicsNode {
public:
  GridNode(cvc::app &ctx, const std::string &statePath, const std::string &name = "grid");
  ~GridNode() override;

  void setBounds(const cvc::bounding_box &bounds);
  // The extent the grid is DRAWN at. Deliberately not getBoundingBox(), which
  // reports an empty box so the grid never contributes to scene bounds — a
  // sensible design that also makes the drawn extent unobservable, so nothing
  // could test whether the grid actually tracks the scene.
  const cvc::bounding_box &bounds() const { return m_bounds; }
  void setColor(double r, double g, double b);

  // Per-plane colors
  void setYZPlaneColor(double r, double g, double b);
  void setXZPlaneColor(double r, double g, double b);
  void setXYPlaneColor(double r, double g, double b);

  void getYZPlaneColor(double &r, double &g, double &b) const;
  void getXZPlaneColor(double &r, double &g, double &b) const;
  void getXYPlaneColor(double &r, double &g, double &b) const;

  // Grid plane visibility (YZ plane at X=0, XZ plane at Y=0, XY plane at Z=0)
  void setYZPlaneVisible(bool visible);
  void setXZPlaneVisible(bool visible);
  void setXYPlaneVisible(bool visible);

  // Hide/show the WHOLE grid — all three plane actors and every tick label, not
  // just the single getProp() actor the base handles.
  void setVisible(bool visible) override;

  bool isYZPlaneVisible() const { return m_yzPlaneVisible; }
  bool isXZPlaneVisible() const { return m_xzPlaneVisible; }
  bool isXYPlaneVisible() const { return m_xyPlaneVisible; }

  // Grid divisions per axis
  void setGridDivisions(int x, int y, int z);
  void getGridDivisions(int &x, int &y, int &z) const;

  // Tick intervals (show tick every N grid cells)
  void setTickIntervals(int x, int y, int z);
  void getTickIntervals(int &x, int &y, int &z) const;

  // Tick label properties
  void setTickLabelColor(double r, double g, double b);
  void getTickLabelColor(double &r, double &g, double &b) const;

  void setTickLabelFontSize(int size);
  int getTickLabelFontSize() const;

  // Override to handle multiple actors
  void addToRenderer(vtkRenderer *renderer) override;
  void removeFromRenderer(vtkRenderer *renderer) override;

  cvc::bounding_box getBoundingBox() const override;

protected:
  vtkProp *getProp() override; // Returns first actor (for compatibility)
  void handleStateChanged(const std::string &childState) override;
  void applyTransformToVTK() override; // Apply transform to all grid actors
  void
  applyClipPlanes(vtkPlaneCollection *planes) override; // Apply clip planes to all grid mappers

private:
  void createGridPlanes();
  void createYZPlane(); // Grid at X=0
  void createXZPlane(); // Grid at Y=0
  void createXYPlane(); // Grid at Z=0

  void createTickLabels();
  void createYZTickLabels();
  void createXZTickLabels();
  void createXYTickLabels();
  void updateTickLabelsInRenderer();

  vtkSmartPointer<vtkActor> m_yzActor; // YZ plane at X=0
  vtkSmartPointer<vtkActor> m_xzActor; // XZ plane at Y=0
  vtkSmartPointer<vtkActor> m_xyActor; // XY plane at Z=0

  vtkSmartPointer<vtkPolyDataMapper> m_yzMapper;
  vtkSmartPointer<vtkPolyDataMapper> m_xzMapper;
  vtkSmartPointer<vtkPolyDataMapper> m_xyMapper;

  // Tick label actors and mappers
  std::vector<vtkSmartPointer<vtkActor2D>> m_yzTickLabelActors;
  std::vector<vtkSmartPointer<vtkActor2D>> m_xzTickLabelActors;
  std::vector<vtkSmartPointer<vtkActor2D>> m_xyTickLabelActors;

  cvc::bounding_box m_bounds;
  int m_divisionsX;
  int m_divisionsY;
  int m_divisionsZ;

  int m_tickIntervalX;
  int m_tickIntervalY;
  int m_tickIntervalZ;

  double m_yzPlaneColor[3];
  double m_xzPlaneColor[3];
  double m_xyPlaneColor[3];

  double m_tickLabelColor[3];
  int m_tickLabelFontSize;

  bool m_yzPlaneVisible;
  bool m_xzPlaneVisible;
  bool m_xyPlaneVisible;

  vtkRenderer *m_renderer; // Track current renderer
};

} // namespace gl
} // namespace cvc

#endif // GRIDNODE_H
