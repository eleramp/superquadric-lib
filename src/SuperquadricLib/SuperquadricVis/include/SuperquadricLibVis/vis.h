#ifndef VTK_H
#define VTK_H

#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

#include <mutex>

namespace SuperqVis {

static std::mutex mtx;

class UpdateCommand : public vtkCommand
{
    const bool *closing;

public:
    vtkTypeMacro(UpdateCommand, vtkCommand);

    static UpdateCommand *New() {return new UpdateCommand;}

    UpdateCommand() : closing(nullptr) { }

    void set_closing(const bool &closing);

    void Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId),
                 void *vtkNotUsed(callData));

};

class Object
{
protected:
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper;
    vtkSmartPointer<vtkActor> vtk_actor;

public:
    vtkSmartPointer<vtkActor> &get_actor();

};

}
#endif