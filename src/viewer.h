/******************************************************************************
 *                                                                            *
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

#ifndef VIEWER_H
#define VIEWER_H

#include <mutex>
#include <vector>
#include <cmath>
#include <limits>
#include <random>

#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkPolyDataMapper.h>
#include <vtkPlaneSource.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkSuperquadric.h>
#include <vtkSampleFunction.h>
#include <vtkContourFilter.h>
#include <vtkArrowSource.h>
#include <vtkTransform.h>
#include <vtkProperty.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>
#include <vtkTextActor.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkSphereSource.h>
#include <vtkSTLReader.h>

#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/PointCloud.h>
#include <yarp/math/Math.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace viewer {

static std::mutex mtx;

/******************************************************************************/
class UpdateCommand : public vtkCommand {
    bool shutdown{false};

public:
    /**************************************************************************/
    vtkTypeMacro(UpdateCommand, vtkCommand);

    /**************************************************************************/
    static UpdateCommand *New() {
        return new UpdateCommand;
    }

    /**************************************************************************/
    void shutDown() {
        shutdown = true;
    }

    /**************************************************************************/
    void Execute(vtkObject* caller, unsigned long vtkNotUsed(eventId),
                 void* vtkNotUsed(callData)) {
        std::lock_guard<std::mutex> lck(mtx);
        vtkRenderWindowInteractor* iren = static_cast<vtkRenderWindowInteractor*>(caller);
        if (shutdown) {
            iren->GetRenderWindow()->Finalize();
            iren->TerminateApp();
        } else {
            iren->Render();
        }
    }
};

/******************************************************************************/
class Viewer {
    vtkSmartPointer<vtkRenderer>                    vtk_renderer{nullptr};
    vtkSmartPointer<vtkRenderWindow>                vtk_renderWindow{nullptr};
    vtkSmartPointer<vtkRenderWindowInteractor>      vtk_renderWindowInteractor{nullptr};
    vtkSmartPointer<UpdateCommand>                  vtk_updateCallback{nullptr};
    vtkSmartPointer<vtkAxesActor>                   vtk_axes{nullptr};
    vtkSmartPointer<vtkInteractorStyleSwitch>       vtk_style{nullptr};
    vtkSmartPointer<vtkCamera>                      vtk_camera{nullptr};
    vtkSmartPointer<vtkPlaneSource>                 vtk_floor{nullptr};
    vtkSmartPointer<vtkPolyDataMapper>              vtk_floor_mapper{nullptr};
    vtkSmartPointer<vtkActor>                       vtk_floor_actor{nullptr};
    vtkSmartPointer<vtkPolyDataMapper>              vtk_object_mapper{nullptr};
    vtkSmartPointer<vtkPoints>                      vtk_object_points{nullptr};
    vtkSmartPointer<vtkUnsignedCharArray>           vtk_object_colors{nullptr};
    vtkSmartPointer<vtkPolyData>                    vtk_object_polydata{nullptr};
    vtkSmartPointer<vtkVertexGlyphFilter>           vtk_object_filter{nullptr};
    vtkSmartPointer<vtkActor>                       vtk_object_actor{nullptr};
    vtkSmartPointer<vtkPolyDataMapper>              vtk_model_mapper{nullptr};
    vtkSmartPointer<vtkActor>                       vtk_model_actor{nullptr};
    vtkSmartPointer<vtkTransform>                   vtk_model_transform{nullptr};
    std::vector<vtkSmartPointer<vtkActor>>          vtk_clusters_actors;

public:
    /**************************************************************************/
    Viewer() = delete;

    /**************************************************************************/
    Viewer(const int x, const int y, const int w, const int h) {
        vtk_renderer = vtkSmartPointer<vtkRenderer>::New();
        vtk_renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
        vtk_renderWindow->SetPosition(x, y);
        vtk_renderWindow->SetSize(w, h);
        vtk_renderWindow->SetWindowName("VTK 3D Viewer");
        vtk_renderWindow->AddRenderer(vtk_renderer);
        vtk_renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
        vtk_renderWindowInteractor->SetRenderWindow(vtk_renderWindow);
        vtk_renderer->SetBackground(std::vector<double>({.7, .7, .7}).data());

        vtk_axes = vtkSmartPointer<vtkAxesActor>::New();
        vtk_axes->GetXAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
        vtk_axes->GetYAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
        vtk_axes->GetZAxisCaptionActor2D()->GetTextActor()->SetTextScaleModeToNone();
        vtk_axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
        vtk_axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
        vtk_axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
        vtk_axes->SetTotalLength(std::vector<double>({.1, .1, .1}).data());
        vtk_renderer->AddActor(vtk_axes);

        vtk_style = vtkSmartPointer<vtkInteractorStyleSwitch>::New();
        vtk_style->SetCurrentStyleToTrackballCamera();
        vtk_renderWindowInteractor->SetInteractorStyle(vtk_style);
    }

    /**************************************************************************/
    void start() {
        vtk_renderWindowInteractor->Initialize();
        vtk_renderWindowInteractor->CreateRepeatingTimer(10);
        vtk_updateCallback = vtkSmartPointer<UpdateCommand>::New();
        vtk_renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, vtk_updateCallback);
        vtk_renderWindowInteractor->Start();
    }

    /**************************************************************************/
    void stop() {
        vtk_updateCallback->shutDown();
    }

    /**************************************************************************/
    void addCamera(const std::vector<double>& position, const std::vector<double>& focalpoint,
                   const std::vector<double>& viewup, const double view_angle) {
        std::lock_guard<std::mutex> lck(mtx);
        vtk_camera = vtkSmartPointer<vtkCamera>::New();
        vtk_camera->SetPosition(position.data());
        vtk_camera->SetFocalPoint(focalpoint.data());
        vtk_camera->SetViewUp(viewup.data());
        vtk_camera->SetViewAngle(view_angle);
        vtk_renderer->SetActiveCamera(vtk_camera);
    }

    /**************************************************************************/
    void addTable(const std::vector<double>& center, const std::vector<double>& normal) {
        std::lock_guard<std::mutex> lck(mtx);
        if (vtk_floor_actor) {
            vtk_renderer->RemoveActor(vtk_floor_actor);
        }

        vtk_floor = vtkSmartPointer<vtkPlaneSource>::New();
        vtk_floor->SetOrigin(0., 0., 0.);
        vtk_floor->SetPoint1(.5, 0., 0.);
        vtk_floor->SetPoint2(0., .5, 0.);
        vtk_floor->SetResolution(20, 20);
        vtk_floor->SetCenter(const_cast<std::vector<double>&>(center).data());
        vtk_floor->SetNormal(const_cast<std::vector<double>&>(normal).data());
        vtk_floor->Update();

        vtk_floor_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_floor_mapper->SetInputData(vtk_floor->GetOutput());
        
        vtk_floor_actor = vtkSmartPointer<vtkActor>::New();
        vtk_floor_actor->SetMapper(vtk_floor_mapper);
        vtk_floor_actor->GetProperty()->SetRepresentationToWireframe();
        
        vtk_renderer->AddActor(vtk_floor_actor);
    }

    /**************************************************************************/
    void addObject(std::shared_ptr<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>> pc) {
        std::lock_guard<std::mutex> lck(mtx);
        if (vtk_object_actor) {
            vtk_renderer->RemoveActor(vtk_object_actor);
        }

        if (vtk_model_actor) {
            vtk_renderer->RemoveActor(vtk_model_actor);
        }

        vtk_object_points = vtkSmartPointer<vtkPoints>::New();
        vtk_object_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        vtk_object_colors->SetNumberOfComponents(3);

        std::vector<unsigned char> color(3);
        for (size_t i = 0; i < pc->size(); i++) {
            const auto& p = (*pc)(i);
            vtk_object_points->InsertNextPoint(p.x, p.y, p.z);

            color = {p.r, p.g, p.b};
            vtk_object_colors->InsertNextTypedTuple(color.data());
        }

        vtk_object_polydata = vtkSmartPointer<vtkPolyData>::New();
        vtk_object_polydata->SetPoints(vtk_object_points);
        vtk_object_polydata->GetPointData()->SetScalars(vtk_object_colors);

        vtk_object_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vtk_object_filter->SetInputData(vtk_object_polydata);
        vtk_object_filter->Update();

        vtk_object_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_object_mapper->SetInputConnection(vtk_object_filter->GetOutputPort());

        vtk_object_actor = vtkSmartPointer<vtkActor>::New();
        vtk_object_actor->SetMapper(vtk_object_mapper);
        vtk_object_actor->GetProperty()->SetPointSize(2);

        vtk_renderer->AddActor(vtk_object_actor);
    }

    /**************************************************************************/
    void addClusters(const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &clusters)
    {
        std::lock_guard<std::mutex> lck(mtx);
        if (!vtk_clusters_actors.empty()) {
            for (auto vtk_actor:vtk_clusters_actors) {
                vtk_renderer->RemoveActor(vtk_actor);
            }
        }

        std::uniform_real_distribution<double> unif(0.0, 1.0);
        std::default_random_engine re;
        for (const auto& c:clusters) {

            vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
            for (size_t i = 0; i < c->size(); i++) {
                const auto& p = (*c)[i];
                vtk_points->InsertNextPoint(p.x, p.y, p.z);
            }

            vtkSmartPointer<vtkPolyData> vtk_polydata = vtkSmartPointer<vtkPolyData>::New();
            vtk_polydata->SetPoints(vtk_points);

            vtkSmartPointer<vtkVertexGlyphFilter> vtk_filter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
            vtk_filter->SetInputData(vtk_polydata);
            vtk_filter->Update();

            vtkSmartPointer<vtkPolyDataMapper> vtk_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            vtk_mapper->SetInputConnection(vtk_filter->GetOutputPort());

            std::vector<double> color{unif(re), unif(re), unif(re)};
            vtkSmartPointer<vtkActor> vtk_actor = vtkSmartPointer<vtkActor>::New();
            vtk_actor->SetMapper(vtk_mapper);
            vtk_actor->GetProperty()->SetColor(color.data());
            vtk_actor->GetProperty()->SetPointSize(2);

            vtk_clusters_actors.push_back(vtk_actor);
            vtk_renderer->AddActor(vtk_actor);
        }
    }   

    /**************************************************************************/
    void addModel(const std::string &model_name, const Eigen::Matrix4d &T,
                  std::vector<double> &color, const double scale,
                  const double &opacity=0.5) {
        std::lock_guard<std::mutex> lck(mtx);       
        vtkSmartPointer<vtkSTLReader> vtk_model_reader;
        vtk_model_reader = vtkSmartPointer<vtkSTLReader>::New();
        vtk_model_reader->SetFileName( model_name.c_str() );
        vtk_model_reader->Update();

        vtk_model_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_model_mapper->SetInputConnection(vtk_model_reader->GetOutputPort());

        vtk_model_actor = vtkSmartPointer<vtkActor>::New();
        vtk_model_actor->SetMapper(vtk_model_mapper);
        vtk_model_actor->GetProperty()->SetColor(color.data());
        vtk_model_actor->GetProperty()->SetOpacity(opacity);

        Eigen::Vector3d v;
        v(0) = T(2,1) - T(1,2);
        v(1) = T(0,2) - T(2,0);
        v(2) = T(1,0) - T(0,1);
        const auto r = std::sqrt(v(0)*v(0) + v(1)*v(1) + v(2)*v(2));
        const auto angle = (180. / M_PI) * std::atan2(.5 * r, .5 * (T(0,0) + T(1,1) + T(2,2) - 1.));

        vtk_model_transform = vtkSmartPointer<vtkTransform>::New();
        vtk_model_transform->Translate(T(0,3), T(1,3), T(2,3));
        vtk_model_transform->RotateWXYZ(angle, v(0)/r, v(1)/r, v(2)/r);
        vtk_model_transform->Scale(scale, scale, scale);
        vtk_model_actor->SetUserTransform(vtk_model_transform);

        vtk_renderer->AddActor(vtk_model_actor);
    }

    /**************************************************************************/
    void clean() {
        std::lock_guard<std::mutex> lck(mtx);
        vtk_renderer->RemoveAllViewProps();
    }

};

}

#endif
