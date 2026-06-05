#include <pybind11/pybind11.h>
#include "PathGuidance.hpp"
namespace py = pybind11;
using namespace rsx;

PYBIND11_MODULE(pathguidance, m) {
    m.doc() = "RSX CanSat 2026 three-phase descent guidance (v1 + replan)";

    py::class_<Vec2>(m, "Vec2").def_readwrite("n", &Vec2::n).def_readwrite("e", &Vec2::e);

    py::enum_<Phase>(m, "Phase")
        .value("Init", Phase::Init).value("Homing", Phase::Homing).value("Loiter", Phase::Loiter)
        .value("Approach", Phase::Approach).value("Landed", Phase::Landed);
    py::enum_<PlanStatus>(m, "PlanStatus")
        .value("Ok", PlanStatus::Ok).value("AdjustedApproach", PlanStatus::AdjustedApproach)
        .value("Infeasible", PlanStatus::Infeasible);
    py::enum_<SegType>(m, "SegType").value("Line", SegType::Line).value("Arc", SegType::Arc);

    py::class_<GuidanceParams>(m, "GuidanceParams")
        .def(py::init<>())
        .def_readwrite("start_n", &GuidanceParams::start_n).def_readwrite("start_e", &GuidanceParams::start_e)
        .def_readwrite("start_d", &GuidanceParams::start_d)
        .def_readwrite("land_n", &GuidanceParams::land_n).def_readwrite("land_e", &GuidanceParams::land_e)
        .def_readwrite("land_d", &GuidanceParams::land_d)
        .def_readwrite("land_heading", &GuidanceParams::land_heading)
        .def_readwrite("glide_ratio", &GuidanceParams::glide_ratio)
        .def_readwrite("approach_len", &GuidanceParams::approach_len)
        .def_readwrite("loiter_radius", &GuidanceParams::loiter_radius)
        .def_readwrite("min_turn_radius", &GuidanceParams::min_turn_radius)
        .def_readwrite("loiter_dir", &GuidanceParams::loiter_dir)
        .def_readwrite("lookahead_drop", &GuidanceParams::lookahead_drop);

    py::class_<State>(m, "State").def(py::init<>())
        .def_readwrite("n", &State::n).def_readwrite("e", &State::e).def_readwrite("d", &State::d)
        .def_readwrite("vn", &State::vn).def_readwrite("ve", &State::ve).def_readwrite("vd", &State::vd)
        .def_readwrite("roll", &State::roll).def_readwrite("pitch", &State::pitch).def_readwrite("yaw", &State::yaw);

    py::class_<HeadingCmd>(m, "HeadingCmd")
        .def_readonly("heading", &HeadingCmd::heading).def_readonly("carrot", &HeadingCmd::carrot)
        .def_readonly("phase", &HeadingCmd::phase).def_readonly("glide_angle", &HeadingCmd::glide_angle)
        .def_readonly("valid", &HeadingCmd::valid);

    py::class_<PathGuidance>(m, "PathGuidance")
        .def(py::init<>()).def(py::init<const GuidanceParams&>())
        .def("set_params", &PathGuidance::setParams)
        .def("plan", &PathGuidance::plan)
        .def("replan", &PathGuidance::replan)
        .def("get_heading", &PathGuidance::getHeading)
        .def("path_at", &PathGuidance::pathAt)
        .def("eval_s", [](const PathGuidance& g, float s){
            float psi=0.f,k=0.f; Vec2 p=g.evalS(s,&psi,&k); return py::make_tuple(p.n,p.e,psi,k); })
        .def("curvature_at", &PathGuidance::curvatureAt)
        .def("total_length", &PathGuidance::totalLength)
        .def("arc_from_alt", &PathGuidance::arcFromAlt)
        .def("seg_count", &PathGuidance::segCount).def("seg_type", &PathGuidance::segType)
        .def("seg_s0", &PathGuidance::segS0).def("seg_len", &PathGuidance::segLen)
        .def("status", &PathGuidance::status)
        .def("entry", &PathGuidance::entry).def("exit", &PathGuidance::exit)
        .def("center", &PathGuidance::center)
        .def("loiter_sweep", &PathGuidance::loiterSweep).def("loiter_turns", &PathGuidance::loiterTurns)
        .def("homing_heading", &PathGuidance::homingHeading)
        .def("d_entry", &PathGuidance::dEntry).def("d_exit", &PathGuidance::dExit)
        .def("resolved_approach_len", &PathGuidance::resolvedApproachLen)
        .def("working_glide_ratio", &PathGuidance::workingGlideRatio)
        .def("budget_residual", &PathGuidance::budgetResidual);
}
