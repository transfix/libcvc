#ifndef PROCEDURALGEOMETRYDIALOG_H
#define PROCEDURALGEOMETRYDIALOG_H

#include <QDialog>
#include <memory>
#include <string>

class QDoubleSpinBox;
class QSpinBox;
class QLabel;
class QDialogButtonBox;
class QVBoxLayout;
class QFormLayout;
class SceneGraph;

// Enum for procedural geometry types
enum class ProceduralGeometryType {
    Sphere,
    Cube,
    Torus,
    Cone
};

class ProceduralGeometryDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ProceduralGeometryDialog(ProceduralGeometryType type, 
                                       std::shared_ptr<SceneGraph> sceneGraph,
                                       QWidget *parent = nullptr);
    ~ProceduralGeometryDialog() override = default;

private slots:
    void onGenerate();

private:
    void setupUI();
    void setupSphereUI(QFormLayout *formLayout);
    void setupCubeUI(QFormLayout *formLayout);
    void setupTorusUI(QFormLayout *formLayout);
    void setupConeUI(QFormLayout *formLayout);
    
    void generateSphere();
    void generateCube();
    void generateTorus();
    void generateCone();
    
    std::string getUniqueName(const std::string& baseName);

    ProceduralGeometryType m_type;
    std::shared_ptr<SceneGraph> m_sceneGraph;
    
    // Common parameters
    QDoubleSpinBox *m_centerXSpinBox;
    QDoubleSpinBox *m_centerYSpinBox;
    QDoubleSpinBox *m_centerZSpinBox;
    
    // Sphere parameters
    QDoubleSpinBox *m_radiusSpinBox;
    QSpinBox *m_thetaResSpinBox;
    QSpinBox *m_phiResSpinBox;
    
    // Cube parameters
    QDoubleSpinBox *m_sizeXSpinBox;
    QDoubleSpinBox *m_sizeYSpinBox;
    QDoubleSpinBox *m_sizeZSpinBox;
    
    // Torus parameters
    QDoubleSpinBox *m_majorRadiusSpinBox;
    QDoubleSpinBox *m_minorRadiusSpinBox;
    QSpinBox *m_majorResSpinBox;
    QSpinBox *m_minorResSpinBox;
    
    // Cone parameters
    QDoubleSpinBox *m_coneRadiusSpinBox;
    QDoubleSpinBox *m_coneHeightSpinBox;
    QSpinBox *m_coneResSpinBox;
    QSpinBox *m_coneCapResSpinBox;
    
    QDialogButtonBox *m_buttonBox;
};

#endif // PROCEDURALGEOMETRYDIALOG_H
