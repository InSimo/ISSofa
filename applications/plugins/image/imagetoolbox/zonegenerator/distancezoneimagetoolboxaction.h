#ifndef DISTANCEZONEIMAGETOOLBOXACTION_H
#define DISTANCEZONEIMAGETOOLBOXACTION_H

#include <QtGui>
#include <QAction>
#include <QGraphicsLineItem>



#include "../labelimagetoolboxaction.h"
//#include "zonegeneratorimagetoolbox.h"

#include "initImage_gui.h"

namespace sofa
{

namespace component
{

namespace engine
{
class SOFA_IMAGE_GUI_API ZoneGeneratorImageToolBoxNoTemplated;
}}}

namespace sofa
{
namespace gui
{
namespace qt
{

class SOFA_IMAGE_GUI_API DistanceZoneImageToolBoxAction : public LabelImageToolBoxAction
{
Q_OBJECT

    //QGraphicsLineItem *lineH[3], *lineV[3];


    QPushButton* generate;
    QLineEdit *seed;
    QLineEdit *density;

public:

    DistanceZoneImageToolBoxAction(sofa::component::engine::LabelImageToolBox* lba,QObject *parent);
    ~DistanceZoneImageToolBoxAction();
    
    sofa::component::engine::ZoneGeneratorImageToolBoxNoTemplated* ZGITB();

    //void setImageSize(int,int,int);
    
private:
    void createMainCommands();

    //QPainterPath drawCursor(double x,double y);
    //QPainterPath drawSegment(VecPixCoord &v, unsigned int axis);
    //void drawSegment();

public slots:
    virtual void addOnGraphs();
    virtual void updateGraphs();
    virtual void updateColor(){}
    virtual void optionChangeSection(sofa::defaulttype::Vec3i);

    
private slots:
    void generatePointButtonClick();
    void selectionPointEvent(int mouseevent, const unsigned int axis,const sofa::defaulttype::Vec3d& imageposition,const sofa::defaulttype::Vec3d& position3D,const QString& value);
    //void sectionButtonClick();
    
    //void positionModified();
    //void radiusModified();
    //void thresholdModified();
    
private:
    
};

}
}
}

#endif // DistanceZoneImageToolBoxAction_H
