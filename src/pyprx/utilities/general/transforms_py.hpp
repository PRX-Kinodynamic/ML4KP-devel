#include <iostream>
#include <boost/python.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <boost/python/return_value_policy.hpp>

#include "prx/utilities/general/transforms.hpp"
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace boost::python;
typedef Eigen::Matrix<ptrdiff_t,1,1>::Index Index;
typedef Eigen::Matrix<double,1,1>::Scalar Scalar; 
typedef Eigen::AngleAxis<double> AngleAxisT;
// enum{Dim=VectorT::RowsAtCompileTime};
// static bool dyn(){ return Dim==Eigen::Dynamic; }

static inline void IDX_CHECK(Index i, Index MAX)
    { if(i<0 || i>=MAX) 
        { PyErr_SetString(PyExc_IndexError,("Index " + boost::lexical_cast<std::string>(i)+" out of range 0.." + boost::lexical_cast<std::string>(MAX-1)).c_str()); throw_error_already_set(); } }
static inline void IDX2_CHECKED_TUPLE_INTS(tuple tuple,const Index max2[2], Index arr2[2]) 
    {Index l=len(tuple); if(l!=2) { PyErr_SetString(PyExc_IndexError,"Index must be integer or a 2-tuple"); throw_error_already_set(); } for(int _i=0; _i<2; _i++) { extract<Index> val(tuple[_i]); if(!val.check()){ PyErr_SetString(PyExc_ValueError,("Unable to convert "+boost::lexical_cast<std::string>(_i)+"-th index to integer.").c_str()); throw_error_already_set(); } Index v=val(); IDX_CHECK(v,max2[_i]); arr2[_i]=v; }  }


template<typename T>
static void set_vector_item(T m, Index ix, Scalar value)
    { IDX_CHECK(ix, (Index) m.size() ); m[ix]=value; }

template<typename T>
static void set_matrix_item(T m, Index ix, Index iy, Scalar value)
    { IDX_CHECK(ix, (Index) m.size() ); m(ix, iy)=value; }

template<typename T>
static Scalar get_item(const T& a, tuple _idx){ Index idx[2]; Index mx[2]={a.rows(),a.cols()}; IDX2_CHECKED_TUPLE_INTS(_idx,mx,idx); return a(idx[0],idx[1]); }
template<typename T>
static void set_item(T& a, tuple _idx, const Scalar& value){ Index idx[2]; Index mx[2]={a.rows(),a.cols()}; IDX2_CHECKED_TUPLE_INTS(_idx,mx,idx); a(idx[0],idx[1])=value; }
template<typename T>
static void set_item_v(T& a, Index _idx, const Scalar& value){ a[_idx]=value; }

template<typename T>
static T transpose(const T& m){ return m.transpose(); }
template<typename T, typename R>
static R diagonal(const T& m){ return m.diagonal(); }

template<typename T>
static T __imul__(T& a, const T& b){ a*=b; return a; };
template<typename T>
static T __mul__(const T& a, const T& b){ return a*b; }
template<typename T, typename R>
static R __mul__vec(const T& m, const R& v){ return m*v; }
// float matrices only
template<typename T>
static T inverse(const T& m){ return m.inverse(); }
template<typename T>
static T __div__(const T& a, const T& b){ return a/b; }

template<typename T>
static T Ones(Index rows, Index cols){     return T::Ones(rows,cols); }
template<typename T>
static T Zero(Index rows, Index cols){     return T::Zero(rows,cols); }
template<typename T>
static T Random(Index rows, Index cols){   return T::Random(rows,cols); }
template<typename T>
static T Identity(Index rows, Index cols){ return T::Identity(rows,cols); }

template<typename T>
static T Zero_1d(Index size){     return T::Zero(size); }

static prx::quaternion_t* fromAxisAngle(const prx::vector_t& axis, const Scalar& angle){ prx::quaternion_t* ret=new prx::quaternion_t(AngleAxisT(angle, axis)); ret->normalize(); return ret; }
static prx::quaternion_t* fromAngleAxis(const Scalar& angle, const prx::vector_t& axis){ prx::quaternion_t* ret=new prx::quaternion_t(AngleAxisT(angle, axis)); ret->normalize(); return ret; }
static prx::quaternion_t* fromTwoVectors(const prx::vector_t& u, const prx::vector_t& v){ prx::quaternion_t* q(new prx::quaternion_t); q->setFromTwoVectors(u,v); return q; }

// template<typename T>
// static std::string to_string()()
// static 
// static auto translation(Eigen::Transform<double, 3, Eigen::AffineCompact> Tr)
//     {return Tr.translation();}
static void translation(prx::transform_t& Tr, prx::vector_t v)
    {Tr.translation() = (v);}
typedef const double& (Eigen::MatrixXd::*parop_signature)(ptrdiff_t,ptrdiff_t) const;

std::string transform_to_str(prx::transform_t obj)
{
    std::ostringstream iss;
    iss << obj.matrix();
  return iss.str();
} 

void pyprx_utilities_general_transforms()
{

    // using n_vector_t = Eigen::Matrix<double, N, 1>;
    // template <int N>
    // using n_matrix_t = Eigen::Matrix<double, N, N>;
    // using vector_t = n_vector_t<3>;
    // using matrix_t = n_matrix_t<3>;
    // using quaternion_t = Eigen::Quaternion<double>;
    // using axis_angle_t = Eigen::AngleAxis<double>;

    class_< prx::n_vector_t<Eigen::Dynamic> >("n_vector", init< prx::n_vector_t<Eigen::Dynamic> >() )
        .def("__setitem__", &set_vector_item< prx::n_vector_t<Eigen::Dynamic> >)
        ;

    class_< prx::n_matrix_t<Eigen::Dynamic> >("n_matrix", init< prx::n_matrix_t<Eigen::Dynamic> >() )
        .def("__setitem__", &set_matrix_item< prx::n_matrix_t<Eigen::Dynamic> >)
        ;

    class_< Eigen::VectorXd >("vector", init< Eigen::VectorXd >() )
        .def("__init__", make_constructor(&init_as_ptr<Eigen::Vector2d,double,double>, default_call_policies(), (args("x"), args("y")) ))
        .def("__init__", make_constructor(&init_as_ptr<prx::vector_t,double,double,double>, default_call_policies(), (args("x"), args("y"), args("z")) ))
        .def("__init__", make_constructor(&init_as_ptr<Eigen::Vector4d,double,double,double,double>, default_call_policies(), (args("x"), args("y"), args("z"), args("w")) ))
        .def("__setitem__", &set_item_v< Eigen::VectorXd >)
        .def("Zero",    &Zero_1d<Eigen::VectorXd>,(arg("size")),"Create zero vector of given dimensions").staticmethod("Zero")
        .def("__str__", &prx_to_str<Eigen::VectorXd>) 
        ;


    class_< Eigen::MatrixXd >("matrix", init< Eigen::MatrixXd >() )
        .def("__call__", static_cast<parop_signature>(&Eigen::MatrixXd::operator()), return_value_policy<copy_const_reference>())
        // .def("__init__", make_constructor(&init_as_ptr<Eigen::Matrix2d,double,double>, default_call_policies() ))
        // .def("__init__", make_constructor(&init_as_ptr<Eigen::Matrix3d,double,double,double>, default_call_policies() ))
        // .def("__init__", make_constructor(&init_as_ptr<Eigen::Matrix4d,double,double,double,double>, default_call_policies() ))
        // .def("__setitem__", &set_matrix_item< Eigen::MatrixXd >)
        .def("__setitem__", &set_item< Eigen::MatrixXd >)
        .def("determinant",&Eigen::MatrixXd::determinant,"Return matrix determinant.")
        .def("trace",&Eigen::MatrixXd::trace,"Return sum of diagonal elements.")
        .def("transpose",&transpose<Eigen::MatrixXd>,"Return transposed matrix.")
        .def("diagonal",&diagonal<Eigen::MatrixXd, Eigen::VectorXd>,"Return diagonal as vector.")
        // // matrix*matrix product
        .def("__mul__",&__mul__<Eigen::MatrixXd>).def("__imul__",&__imul__<Eigen::MatrixXd>)
        // // matrix*vector product
        .def("__mul__",&__mul__vec<Eigen::MatrixXd, Eigen::VectorXd>).def("__rmul__",&__mul__vec<Eigen::MatrixXd, Eigen::VectorXd>)
        .def("Zero",    &Zero<Eigen::MatrixXd>,(arg("rows"),arg("cols")),"Create zero matrix of given dimensions").staticmethod("Zero")
        .def("Ones",    &Ones<Eigen::MatrixXd>,(arg("rows"),arg("cols")),"Create matrix of given dimensions where all elements are set to 1.").staticmethod("Ones")
        .def("Random",  &Random<Eigen::MatrixXd>,(arg("rows"),arg("cols")),"Create matrix with given dimensions where all elements are set to number between 0 and 1 (uniformly-distributed).").staticmethod("Random")
        .def("Identity",&Identity<Eigen::MatrixXd>,(arg("rows"),arg("cols")),"Create identity matrix with given rows anc columns.").staticmethod("Identity")
        // .def("__setitem__",&prx::matrix_t::set_row).def("__getitem__",&prx::matrix_t::get_row)
        // .def("__setitem__",&prx::matrix_t::set_item< prx::matrix_t >).def("__getitem__",&prx::matrix_t::get_item< prx::matrix_t >)
        // .def(str(self))
        .def(self_ns::str(self_ns::self))
        .def("__str__", &prx_to_str<Eigen::MatrixXd>) 

        // .def("__str__",&Eigen::MatrixXd::operator<<)
        // .def("__repr__",&prx::matrix_t::__str__)
        ;

    class_< prx::quaternion_t >("quaternion" )
        .def("__init__", make_constructor(&fromAxisAngle, default_call_policies(),(arg("axis"),  arg("angle"))))
        .def("__init__", make_constructor(&fromAngleAxis, default_call_policies(),(arg("angle"), arg("axis"))))
        .def("__init__", make_constructor(&fromTwoVectors,default_call_policies(),(arg("u"),     arg("v"))))
        .def(init<Scalar,Scalar,Scalar,Scalar>((arg("w"),arg("x"),arg("y"),arg("z")),"Initialize from coefficients.\n\n.. note:: The order of coefficients is *w*, *x*, *y*, *z*. The [] operator numbers them differently, 0...4 for *x* *y* *z* *w*!"))
        .def(init<prx::matrix_t>((arg("rotMatrix")))) //,"Initialize from given rotation matrix.")
        .def(init<prx::quaternion_t>((arg("other"))))
        ;

    class_<prx::transform_t, std::shared_ptr<prx::transform_t>>("transform")
        // .def("__init__", make_constructor(&fromAxisAngle, default_call_policies(),(arg("axis"),  arg("angle"))))
        .def("setIdentity", &prx::transform_t::setIdentity)
        .def("translation", &translation)
        .def("__str__", &transform_to_str) 
        ;



}
