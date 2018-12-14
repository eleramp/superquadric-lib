#include "vtkSuperq.h"

#include <cmath>

using namespace std;
using namespace Eigen;
using namespace SuperqVis;


/**********************************************/
Superquadric::Superquadric(const VectorXd &r)
{
    double bx=2.0*r(7);
    double by=2.0*r(8);
    double bz=2.0*r(9);

    vtk_superquadric=vtkSmartPointer<vtkSuperquadric>::New();
    vtk_superquadric->ToroidalOff();
    vtk_superquadric->SetSize(1.0);
    vtk_superquadric->SetCenter(Zero(3));

    vtk_superquadric->SetScale(r(7),r(8),r(9));
    vtk_superquadric->SetPhiRoundness(r(10);
    vtk_superquadric->SetThetaRoundness(r(11));

    vtk_sample=vtkSmartPointer<vtkSampleFunction>::New();
    vtk_sample->SetSampleDimensions(50,50,50);
    vtk_sample->SetImplicitFunction(vtk_superquadric);
    vtk_sample->SetModelBounds(-bx,bx,-by,by,-bz,bz);

    // The isosurface is defined at 0.0 as specified in
    // https://github.com/Kitware/VTK/blob/master/Common/DataModel/vtkSuperquadric.cxx
    vtk_contours=vtkSmartPointer<vtkContourFilter>::New();
    vtk_contours->SetInputConnection(vtk_sample->GetOutputPort());
    vtk_contours->GenerateValues(1,0.0,0.0);

    vtk_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
    vtk_mapper->SetInputConnection(vtk_contours->GetOutputPort());
    vtk_mapper->ScalarVisibilityOff();

    vtk_actor=vtkSmartPointer<vtkActor>::New();
    vtk_actor->SetMapper(vtk_mapper);
    vtk_actor->GetProperty()->SetColor(0.0,0.3,0.6);
    vtk_actor->GetProperty()->SetOpacity(0.25);

    vtk_transform=vtkSmartPointer<vtkTransform>::New();
    vtk_transform->Translate(r.subVector(0,2).data());
    vtk_transform->RotateWXYZ((180.0/M_PI)*r(6),r.segment(3,5));
    vtk_actor->SetUserTransform(vtk_transform);
}

/**********************************************/
void Superquadric::set_parameters(const Vector &r)
{
    double bx=2.0*r(7);
    double by=2.0*r(8);
    double bz=2.0*r(9);

    vtk_superquadric->SetScale(r(7),r(8),r(9));
    vtk_superquadric->SetPhiRoundness(r(10);
    vtk_superquadric->SetThetaRoundness(r(11));

    vtk_sample->SetModelBounds(-bx,bx,-by,by,-bz,bz);

    vtk_transform->Identity();
    vtk_transform->Translate(r.subVector(0,2).data());
    vtk_transform->RotateWXYZ((180.0/M_PI)*r(6),r.segment(3,5));
}
