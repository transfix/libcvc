#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>
#include <cmath>
#include <cvc/core/state.h>
#include <cvc/geometry/geometry.h>
#include <cvc/utility/algorithm.h>
#include <volrover3/AppState.h>
#include <volrover3/GeometryNode.h>
#include <volrover3/ProceduralGeometryDialog.h>
#include <volrover3/SceneGraph.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

ProceduralGeometryDialog::ProceduralGeometryDialog(ProceduralGeometryType type,
                                                   std::shared_ptr<SceneGraph> sceneGraph,
                                                   QWidget *parent)
    : QDialog(parent), m_type(type), m_sceneGraph(sceneGraph), m_centerXSpinBox(nullptr),
      m_centerYSpinBox(nullptr), m_centerZSpinBox(nullptr), m_radiusSpinBox(nullptr),
      m_thetaResSpinBox(nullptr), m_phiResSpinBox(nullptr), m_sizeXSpinBox(nullptr),
      m_sizeYSpinBox(nullptr), m_sizeZSpinBox(nullptr), m_majorRadiusSpinBox(nullptr),
      m_minorRadiusSpinBox(nullptr), m_majorResSpinBox(nullptr), m_minorResSpinBox(nullptr),
      m_coneRadiusSpinBox(nullptr), m_coneHeightSpinBox(nullptr), m_coneResSpinBox(nullptr),
      m_coneCapResSpinBox(nullptr), m_buttonBox(nullptr) {
  setupUI();
}

void ProceduralGeometryDialog::setupUI() {
  QString title;
  switch (m_type) {
  case ProceduralGeometryType::Sphere:
    title = tr("Generate Sphere");
    break;
  case ProceduralGeometryType::Cube:
    title = tr("Generate Cube");
    break;
  case ProceduralGeometryType::Torus:
    title = tr("Generate Torus");
    break;
  case ProceduralGeometryType::Cone:
    title = tr("Generate Cone");
    break;
  }
  setWindowTitle(title);

  QVBoxLayout *mainLayout = new QVBoxLayout(this);

  // Center position group
  QGroupBox *centerGroup = new QGroupBox(tr("Center Position"), this);
  QFormLayout *centerLayout = new QFormLayout(centerGroup);

  m_centerXSpinBox = new QDoubleSpinBox(this);
  m_centerXSpinBox->setRange(-1000.0, 1000.0);
  m_centerXSpinBox->setValue(0.0);
  m_centerXSpinBox->setDecimals(3);
  centerLayout->addRow(tr("X:"), m_centerXSpinBox);

  m_centerYSpinBox = new QDoubleSpinBox(this);
  m_centerYSpinBox->setRange(-1000.0, 1000.0);
  m_centerYSpinBox->setValue(0.0);
  m_centerYSpinBox->setDecimals(3);
  centerLayout->addRow(tr("Y:"), m_centerYSpinBox);

  m_centerZSpinBox = new QDoubleSpinBox(this);
  m_centerZSpinBox->setRange(-1000.0, 1000.0);
  m_centerZSpinBox->setValue(0.0);
  m_centerZSpinBox->setDecimals(3);
  centerLayout->addRow(tr("Z:"), m_centerZSpinBox);

  mainLayout->addWidget(centerGroup);

  // Parameters group
  QGroupBox *paramsGroup = new QGroupBox(tr("Parameters"), this);
  QFormLayout *paramsLayout = new QFormLayout(paramsGroup);

  switch (m_type) {
  case ProceduralGeometryType::Sphere:
    setupSphereUI(paramsLayout);
    break;
  case ProceduralGeometryType::Cube:
    setupCubeUI(paramsLayout);
    break;
  case ProceduralGeometryType::Torus:
    setupTorusUI(paramsLayout);
    break;
  case ProceduralGeometryType::Cone:
    setupConeUI(paramsLayout);
    break;
  }

  mainLayout->addWidget(paramsGroup);

  // Buttons
  m_buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
  m_buttonBox->button(QDialogButtonBox::Ok)->setText(tr("Generate"));
  connect(m_buttonBox, &QDialogButtonBox::accepted, this, &ProceduralGeometryDialog::onGenerate);
  connect(m_buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
  mainLayout->addWidget(m_buttonBox);

  setMinimumWidth(300);
}

void ProceduralGeometryDialog::setupSphereUI(QFormLayout *formLayout) {
  m_radiusSpinBox = new QDoubleSpinBox(this);
  m_radiusSpinBox->setRange(0.001, 1000.0);
  m_radiusSpinBox->setValue(1.0);
  m_radiusSpinBox->setDecimals(3);
  m_radiusSpinBox->setToolTip(tr("Radius of the sphere"));
  formLayout->addRow(tr("Radius:"), m_radiusSpinBox);

  m_thetaResSpinBox = new QSpinBox(this);
  m_thetaResSpinBox->setRange(3, 256);
  m_thetaResSpinBox->setValue(32);
  m_thetaResSpinBox->setToolTip(tr("Number of segments around the equator"));
  formLayout->addRow(tr("Theta Resolution:"), m_thetaResSpinBox);

  m_phiResSpinBox = new QSpinBox(this);
  m_phiResSpinBox->setRange(3, 256);
  m_phiResSpinBox->setValue(16);
  m_phiResSpinBox->setToolTip(tr("Number of segments from pole to pole"));
  formLayout->addRow(tr("Phi Resolution:"), m_phiResSpinBox);
}

void ProceduralGeometryDialog::setupCubeUI(QFormLayout *formLayout) {
  m_sizeXSpinBox = new QDoubleSpinBox(this);
  m_sizeXSpinBox->setRange(0.001, 1000.0);
  m_sizeXSpinBox->setValue(1.0);
  m_sizeXSpinBox->setDecimals(3);
  m_sizeXSpinBox->setToolTip(tr("Size along X axis"));
  formLayout->addRow(tr("Width (X):"), m_sizeXSpinBox);

  m_sizeYSpinBox = new QDoubleSpinBox(this);
  m_sizeYSpinBox->setRange(0.001, 1000.0);
  m_sizeYSpinBox->setValue(1.0);
  m_sizeYSpinBox->setDecimals(3);
  m_sizeYSpinBox->setToolTip(tr("Size along Y axis"));
  formLayout->addRow(tr("Height (Y):"), m_sizeYSpinBox);

  m_sizeZSpinBox = new QDoubleSpinBox(this);
  m_sizeZSpinBox->setRange(0.001, 1000.0);
  m_sizeZSpinBox->setValue(1.0);
  m_sizeZSpinBox->setDecimals(3);
  m_sizeZSpinBox->setToolTip(tr("Size along Z axis"));
  formLayout->addRow(tr("Depth (Z):"), m_sizeZSpinBox);
}

void ProceduralGeometryDialog::setupTorusUI(QFormLayout *formLayout) {
  m_majorRadiusSpinBox = new QDoubleSpinBox(this);
  m_majorRadiusSpinBox->setRange(0.001, 1000.0);
  m_majorRadiusSpinBox->setValue(1.0);
  m_majorRadiusSpinBox->setDecimals(3);
  m_majorRadiusSpinBox->setToolTip(tr("Distance from center to tube center"));
  formLayout->addRow(tr("Major Radius:"), m_majorRadiusSpinBox);

  m_minorRadiusSpinBox = new QDoubleSpinBox(this);
  m_minorRadiusSpinBox->setRange(0.001, 1000.0);
  m_minorRadiusSpinBox->setValue(0.25);
  m_minorRadiusSpinBox->setDecimals(3);
  m_minorRadiusSpinBox->setToolTip(tr("Radius of the tube"));
  formLayout->addRow(tr("Minor Radius:"), m_minorRadiusSpinBox);

  m_majorResSpinBox = new QSpinBox(this);
  m_majorResSpinBox->setRange(3, 256);
  m_majorResSpinBox->setValue(32);
  m_majorResSpinBox->setToolTip(tr("Number of segments around the torus"));
  formLayout->addRow(tr("Major Resolution:"), m_majorResSpinBox);

  m_minorResSpinBox = new QSpinBox(this);
  m_minorResSpinBox->setRange(3, 256);
  m_minorResSpinBox->setValue(16);
  m_minorResSpinBox->setToolTip(tr("Number of segments around the tube"));
  formLayout->addRow(tr("Minor Resolution:"), m_minorResSpinBox);
}

void ProceduralGeometryDialog::setupConeUI(QFormLayout *formLayout) {
  m_coneRadiusSpinBox = new QDoubleSpinBox(this);
  m_coneRadiusSpinBox->setRange(0.001, 1000.0);
  m_coneRadiusSpinBox->setValue(0.5);
  m_coneRadiusSpinBox->setDecimals(3);
  m_coneRadiusSpinBox->setToolTip(tr("Radius of the cone base"));
  formLayout->addRow(tr("Base Radius:"), m_coneRadiusSpinBox);

  m_coneHeightSpinBox = new QDoubleSpinBox(this);
  m_coneHeightSpinBox->setRange(0.001, 1000.0);
  m_coneHeightSpinBox->setValue(1.0);
  m_coneHeightSpinBox->setDecimals(3);
  m_coneHeightSpinBox->setToolTip(tr("Height of the cone"));
  formLayout->addRow(tr("Height:"), m_coneHeightSpinBox);

  m_coneResSpinBox = new QSpinBox(this);
  m_coneResSpinBox->setRange(3, 256);
  m_coneResSpinBox->setValue(32);
  m_coneResSpinBox->setToolTip(tr("Number of segments around the cone"));
  formLayout->addRow(tr("Resolution:"), m_coneResSpinBox);

  m_coneCapResSpinBox = new QSpinBox(this);
  m_coneCapResSpinBox->setRange(1, 64);
  m_coneCapResSpinBox->setValue(1);
  m_coneCapResSpinBox->setToolTip(tr("Number of rings on the base cap"));
  formLayout->addRow(tr("Cap Resolution:"), m_coneCapResSpinBox);
}

void ProceduralGeometryDialog::onGenerate() {
  try {
    switch (m_type) {
    case ProceduralGeometryType::Sphere:
      generateSphere();
      break;
    case ProceduralGeometryType::Cube:
      generateCube();
      break;
    case ProceduralGeometryType::Torus:
      generateTorus();
      break;
    case ProceduralGeometryType::Cone:
      generateCone();
      break;
    }
    accept();
  } catch (const std::exception &e) {
    QMessageBox::critical(this, tr("Generation Error"),
                          tr("Failed to generate geometry:\n%1").arg(e.what()));
  }
}

std::string ProceduralGeometryDialog::getUniqueName(const std::string &baseName) {
  std::string name = baseName;
  int counter = 1;
  while (m_sceneGraph->getGraphics(name)) {
    name = baseName + "_" + std::to_string(counter++);
  }
  return name;
}

void ProceduralGeometryDialog::generateSphere() {
  double cx = m_centerXSpinBox->value();
  double cy = m_centerYSpinBox->value();
  double cz = m_centerZSpinBox->value();
  double radius = m_radiusSpinBox->value();
  int thetaRes = m_thetaResSpinBox->value();
  int phiRes = m_phiResSpinBox->value();

  // Use the algorithm function to generate the geometry
  cvc::geometry geom = cvc::generate_sphere(cx, cy, cz, radius, thetaRes, phiRes);

  // Create geometry node
  std::string name = getUniqueName("Sphere");
  auto node = m_sceneGraph->getGraphicsRoot()->addGraphicsChild<GeometryNode>(name);
  m_sceneGraph->registerGraphics(name, node);

  node->setGeometry(geom);
  node->setMetadata("type", std::string("geometry"));
  node->setMetadata("source", std::string("procedural"));
  node->setMetadata("primitive", std::string("sphere"));
  node->setMetadata("num_vertices", static_cast<int>(geom.num_points()));
  node->setMetadata("num_triangles", static_cast<int>(geom.num_tris()));

  AppState::instance().setWorldBounds(geom.extents());
}

void ProceduralGeometryDialog::generateCube() {
  double cx = m_centerXSpinBox->value();
  double cy = m_centerYSpinBox->value();
  double cz = m_centerZSpinBox->value();
  double sx = m_sizeXSpinBox->value();
  double sy = m_sizeYSpinBox->value();
  double sz = m_sizeZSpinBox->value();

  // Use the algorithm function to generate the geometry
  cvc::geometry geom = cvc::generate_cube(cx, cy, cz, sx, sy, sz);

  // Create geometry node
  std::string name = getUniqueName("Cube");
  auto node = m_sceneGraph->getGraphicsRoot()->addGraphicsChild<GeometryNode>(name);
  m_sceneGraph->registerGraphics(name, node);

  node->setGeometry(geom);
  node->setMetadata("type", std::string("geometry"));
  node->setMetadata("source", std::string("procedural"));
  node->setMetadata("primitive", std::string("cube"));
  node->setMetadata("num_vertices", static_cast<int>(geom.num_points()));
  node->setMetadata("num_triangles", static_cast<int>(geom.num_tris()));

  AppState::instance().setWorldBounds(geom.extents());
}

void ProceduralGeometryDialog::generateTorus() {
  double cx = m_centerXSpinBox->value();
  double cy = m_centerYSpinBox->value();
  double cz = m_centerZSpinBox->value();
  double majorRadius = m_majorRadiusSpinBox->value();
  double minorRadius = m_minorRadiusSpinBox->value();
  int majorRes = m_majorResSpinBox->value();
  int minorRes = m_minorResSpinBox->value();

  // Use the algorithm function to generate the geometry
  cvc::geometry geom =
      cvc::generate_torus(cx, cy, cz, majorRadius, minorRadius, majorRes, minorRes);

  // Create geometry node
  std::string name = getUniqueName("Torus");
  auto node = m_sceneGraph->getGraphicsRoot()->addGraphicsChild<GeometryNode>(name);
  m_sceneGraph->registerGraphics(name, node);

  node->setGeometry(geom);
  node->setMetadata("type", std::string("geometry"));
  node->setMetadata("source", std::string("procedural"));
  node->setMetadata("primitive", std::string("torus"));
  node->setMetadata("num_vertices", static_cast<int>(geom.num_points()));
  node->setMetadata("num_triangles", static_cast<int>(geom.num_tris()));

  AppState::instance().setWorldBounds(geom.extents());
}

void ProceduralGeometryDialog::generateCone() {
  double cx = m_centerXSpinBox->value();
  double cy = m_centerYSpinBox->value();
  double cz = m_centerZSpinBox->value();
  double radius = m_coneRadiusSpinBox->value();
  double height = m_coneHeightSpinBox->value();
  int res = m_coneResSpinBox->value();

  // Use the algorithm function to generate the geometry
  // Note: The algorithm function doesn't use capRes parameter
  cvc::geometry geom = cvc::generate_cone(cx, cy, cz, radius, height, res);

  // Create geometry node
  std::string name = getUniqueName("Cone");
  auto node = m_sceneGraph->getGraphicsRoot()->addGraphicsChild<GeometryNode>(name);
  m_sceneGraph->registerGraphics(name, node);

  node->setGeometry(geom);
  node->setMetadata("type", std::string("geometry"));
  node->setMetadata("source", std::string("procedural"));
  node->setMetadata("primitive", std::string("cone"));
  node->setMetadata("num_vertices", static_cast<int>(geom.num_points()));
  node->setMetadata("num_triangles", static_cast<int>(geom.num_tris()));

  AppState::instance().setWorldBounds(geom.extents());
}
