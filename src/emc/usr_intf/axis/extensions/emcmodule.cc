//    This is a component of AXIS, a front-end for LinuxCNC
//    Copyright 2004, 2005, 2006 Jeff Epler <jepler@unpythonic.net> and
//    Chris Radek <chris@timeguy.com>
//    Copyright 2026 B.Stultiens
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

#define PY_SSIZE_T_CLEAN
// SyncTwin no-nml: the module keeps the INI reader (linuxcnc.ini), the constants, the tool struct and
// the backplot geometry helpers. The NML clients stat / command / error_channel and positionlogger (it
// reads stat) are gone with NML - the task's bus is NATS (emc/task/tasknats.cc): status = Sparkplug
// DDATA, commands = DCMD.
#include <stdio.h>
#include <stdlib.h>
#include <Python.h>
#include <structseq.h>
#include <pthread.h>
#include <structmember.h>
#include "config.h"
#include "libnml/rcs/rcs.hh"
#include "nml_intf/emc.hh"
#include "nml_intf/emc_nml.hh"
#include "nml_intf/debugflags.h"
#include <kinematics.h>
#include "config.h"
#include <inifile.hh>
#include "libnml/rcs/rcs_print.hh"
#include <sys/types.h>
#include <unistd.h>

#include "tooldata/tooldata.hh"

#include <cmath>

#include <algorithm>

using namespace linuxcnc;

#define LOCAL_SPINDLE_FORWARD (1)
#define LOCAL_SPINDLE_REVERSE (-1)
#define LOCAL_SPINDLE_OFF (0)
#define LOCAL_SPINDLE_INCREASE (10)
#define LOCAL_SPINDLE_DECREASE (11)
#define LOCAL_SPINDLE_CONSTANT (12)

#define LOCAL_MIST_ON (1)
#define LOCAL_MIST_OFF (0)

#define LOCAL_FLOOD_ON (1)
#define LOCAL_FLOOD_OFF (0)

#define LOCAL_BRAKE_ENGAGE (1)
#define LOCAL_BRAKE_RELEASE (0)

#define LOCAL_JOG_STOP (0)
#define LOCAL_JOG_CONTINUOUS (1)
#define LOCAL_JOG_INCREMENT (2)

#define LOCAL_AUTO_RUN (0)
#define LOCAL_AUTO_PAUSE (1)
#define LOCAL_AUTO_RESUME (2)
#define LOCAL_AUTO_STEP (3)
#define LOCAL_AUTO_REVERSE (4)
#define LOCAL_AUTO_FORWARD (5)

struct pyIniFile {
    PyObject_HEAD
    // The 'inifile' member is a pointer because we don't
    // have C++ constructor semantics here
    std::string *inifile;
};


static PyObject *m = NULL, *error = NULL;

static int Ini_init(pyIniFile *self, PyObject *a, PyObject * /*k*/) {
    const char *inifile = NULL;
    if(!PyArg_ParseTuple(a, "s", &inifile)) return -1;

    if(!inifile)
        return -1;

    self->inifile = new std::string(inifile);

    IniFile ini(self->inifile); // Test open (will parse and cache the file)
    if (!ini) {
        PyErr_Format( error, "inifile.open(%s) failed", inifile);
        return -1;
    }
    return 0;
}

//
// PyBool linuxcnc.ini.hasvariable(string:section, string:variable [, int:num])
//
// Find [section]variable and return true if found. The 'section' may be an
// empty string and the first occurrence of 'variable' in any section is
// searched.
//
static PyObject *Ini_has_variable(pyIniFile *self, PyObject *args)
{
    const char *sect, *var;
    int num = 1;
    if(!PyArg_ParseTuple(args, "ss|i:hasvariable", &sect, &var, &num))
        return NULL;

    IniFile ini(self->inifile);
    if(!ini) {
        PyErr_Format(error, "Internal: ini-file could not be reopened");
        return NULL;
    }

    return PyBool_FromLong((long)ini.hasVariable(num, var, sect));
}

//
// PyBool linuxcnc.ini.hassection(string:section)
//
// Find [section] and return true if found.
//
static PyObject *Ini_has_section(pyIniFile *self, PyObject *args)
{
    const char *sect;
    if(!PyArg_ParseTuple(args, "s:hassection", &sect))
        return NULL;

    IniFile ini(self->inifile);
    if(!ini) {
        PyErr_Format(error, "Internal: ini-file could not be reopened");
        return NULL;
    }

    return PyBool_FromLong((long)ini.hasSection(sect));
}

//
// The prototype of PyArg_ParseTupleAndKeywords() was originally using char**
// for the keyword array. This is obviously problematic and was changed in 3.13
// to const. C++ will generate an error when demoting const to non-const
// pointers. We need to support multiple python versions and make this
// patchwork to support them all.
#if PY_VERSION_HEX >= 0x030d00f0	// 3.13
#define PARSE_KW_CONST const
#else
#define PARSE_KW_CONST
#endif

static PARSE_KW_CONST char kw_empty[] = "";
static PARSE_KW_CONST char kw_fallback[] = "fallback";
static PARSE_KW_CONST char *kw_eeefn[] = { kw_empty, kw_empty, kw_empty, kw_fallback, NULL };
static PARSE_KW_CONST char *kw_efn[]   = { kw_empty, kw_fallback, NULL };

#undef PARSE_KW_CONST

//
// PyBool|None linuxcnc.ini.getbool(string:section, string:variable [, int:num] [, fallback=val])
//
// Find [section]variable and convert it to boolean. Valid boolean values are
// (case insensitive) {true, yes, on, 1} and {false, no, off, 0}.
// The optional named argument fallback= defines the object to return when the
// variable is not found or invalid. The optional named option num= selects the
// num'th variable of the section (default to 1).
//
static PyObject *Ini_get_bool(pyIniFile *self, PyObject *args, PyObject *kwargs)
{
    const char *sect, *var;
    int num = 1;
    PyObject *def = Py_None;
    if(!PyArg_ParseTupleAndKeywords(args, kwargs, "ss|i$O:getbool", kw_eeefn, &sect, &var, &num, &def))
        return NULL;

    if(num < 1) {
        PyErr_Format(error, "Argument 'num' must be >= 1");
        return NULL;
    }

    IniFile ini(self->inifile);
    if(!ini) {
        PyErr_Format(error, "Internal: ini-file could not be reopened");
        return NULL;
    }

    if(auto v = ini.findBool(num, var, sect))
        return PyBool_FromLong(*v);

    // Not found or error
    // We have a fallback set by argument or as None
    Py_INCREF(def);
    return def;
}

//
// PyLong|None linuxcnc.ini.getsint(string:section, string:variable [, int:num] [, fallback=val])
//
// Find [section]variable and convert it to a signed integer.
// The optional named argument fallback= defines the value to return when the
// variable is not found or invalid. The optional named option num= selects the
// num'th variable of the section (default to 1).
//
static PyObject *Ini_get_sint(pyIniFile *self, PyObject *args, PyObject *kwargs)
{
    const char *sect, *var;
    int num = 1;
    PyObject *def = Py_None;
    if(!PyArg_ParseTupleAndKeywords(args, kwargs, "ss|i$O:getsint", kw_eeefn, &sect, &var, &num, &def))
        return NULL;

    if(num < 1) {
        PyErr_Format(error, "Argument 'num' must be >= 1");
        return NULL;
    }

    IniFile ini(self->inifile);
    if(!ini) {
        PyErr_Format(error, "Internal: ini-file could not be reopened");
        return NULL;
    }

    if(auto v = ini.findSInt(num, var, sect))
        return PyLong_FromLongLong(*v);

    // Not found or error
    // We have a fallback set by argument or as None
    Py_INCREF(def);
    return def;
}

//
// PyLong|None linuxcnc.ini.getuint(string:section, string:variable [, int:num] [, fallback=val])
//
// Find [section]variable and convert it to an unsigned integer.
// The optional named argument fallback= defines the value to return when the
// variable is not found or invalid. The optional named option num= selects the
// num'th variable of the section (default to 1).
//
static PyObject *Ini_get_uint(pyIniFile *self, PyObject *args, PyObject *kwargs)
{
    const char *sect, *var;
    int num = 1;
    PyObject *def = Py_None;
    if(!PyArg_ParseTupleAndKeywords(args, kwargs, "ss|i$O:getuint", kw_eeefn, &sect, &var, &num, &def))
        return NULL;

    if(num < 1) {
        PyErr_Format(error, "Argument 'num' must be >= 1");
        return NULL;
    }

    IniFile ini(self->inifile);
    if(!ini) {
        PyErr_Format(error, "Internal: ini-file could not be reopened");
        return NULL;
    }

    if(auto v = ini.findUInt(num, var, sect))
        return PyLong_FromUnsignedLongLong(*v);

    // Not found or error
    // We have a fallback set by argument or as None
    Py_INCREF(def);
    return def;
}

//
// PyFloat|None linuxcnc.ini.getreal(string:section, string:variable [, int:num] [, fallback=val])
//
// Find [section]variable and convert it to an unsigned integer.
// The optional named argument fallback= defines the value to return when the
// variable is not found or invalid. The optional named option num= selects the
// num'th variable of the section (default to 1).
//
static PyObject *Ini_get_real(pyIniFile *self, PyObject *args, PyObject *kwargs)
{
    const char *sect, *var;
    int num = 1;
    PyObject *def = Py_None;
    if(!PyArg_ParseTupleAndKeywords(args, kwargs, "ss|i$O:getreal", kw_eeefn, &sect, &var, &num, &def))
        return NULL;

    if(num < 1) {
        PyErr_Format(error, "Argument 'num' must be >= 1");
        return NULL;
    }

    IniFile ini(self->inifile);
    if(!ini) {
        PyErr_Format(error, "Internal: ini-file could not be reopened");
        return NULL;
    }

    if(auto v = ini.findReal(num, var, sect))
        return PyFloat_FromDouble(*v);

    // Not found or error
    // We have a fallback set by argument or as None
    Py_INCREF(def);
    return def;
}

//
// PyString|None linuxcnc.ini.getstring(string:section, string:variable [, int:num] [, fallback=val])
//
// Find [section]variable and convert it to a string.
// The optional named argument fallback= defines the value to return when the
// variable is not found or invalid. The optional named option num= selects the
// num'th variable of the section (default to 1).
//
static PyObject *Ini_get_string(pyIniFile *self, PyObject *args, PyObject *kwargs)
{
    const char *sect, *var;
    int num = 1;
    PyObject *def = Py_None;
    if(!PyArg_ParseTupleAndKeywords(args, kwargs, "ss|i$O:getstring", kw_eeefn, &sect, &var, &num, &def))
        return NULL;

    if(num < 1) {
        PyErr_Format(error, "Argument 'num' must be >= 1");
        return NULL;
    }

    IniFile ini(self->inifile);
    if(!ini) {
        PyErr_Format(error, "Internal: ini-file could not be reopened");
        return NULL;
    }

    if(auto v = ini.findString(num, var, sect))
        return PyUnicode_FromString(v->c_str());

    // Not found or error
    // We have a fallback set by argument or as None
    Py_INCREF(def);
    return def;
}

//
// PyList(PyTuple(variable,value)) linuxcnc.ini.getsection([string:section])
//
// Returns a list of tuples containing variable name and value from the named
// section or all variables from all sections if no section given.
//
static PyObject *Ini_get_variables(pyIniFile *self, PyObject *args)
{
    const char *sect = "";
    if(!PyArg_ParseTuple(args, "|s:getvariables", &sect)) return NULL;

    IniFile ini(self->inifile);
    if(!ini) {
        PyErr_Format(error, "Internal: ini-file could not be reopened");
        return NULL;
    }

    PyObject *list = PyList_New(0);
    if(!list)
        return NULL;

    for(auto const &v : ini.findVariables(sect)) {
        PyObject *var = PyUnicode_FromString(v.first.c_str());
        if(!var) {
            Py_DECREF(list);
            return NULL;
        }
        PyObject *val = PyUnicode_FromString(v.second.c_str());
        if(!val) {
            Py_DECREF(var);
            Py_DECREF(list);
            return NULL;
        }
        PyObject *tup = PyTuple_New(2);
        if(!tup) {
            Py_DECREF(val);
            Py_DECREF(var);
            Py_DECREF(list);
            return NULL;
        }
        PyTuple_SET_ITEM(tup, 0, var);
        PyTuple_SET_ITEM(tup, 1, val);
        PyList_Append(list, tup);
    }
    return list;
}

//
// PyList linuxcnc.ini.sections()
//
// Returns a list of section names from the ini-file.
//
static PyObject *Ini_get_sections(pyIniFile *self, PyObject *)
{
    IniFile ini(self->inifile);
    if(!ini) {
        PyErr_Format(error, "Internal: ini-file could not be reopened");
        return NULL;
    }

    PyObject *list = PyList_New(0);
    if(!list)
        return NULL;

    for(auto const &v : ini.findSections()) {
        PyObject *val = PyUnicode_FromString(v.c_str());
        if(!val) {
            Py_DECREF(list);
            return NULL;
        }
        PyList_Append(list, val);
    }
    return list;
}

//
// PyList linuxcnc.ini.findall(string:section [, string:variable])
//
// Return a list of [section]variable entries where the variable name is the
// same for all entries found. If the section string is empty (''), then all
// variables of that name are returned.
//
static PyObject *Ini_findall(pyIniFile *self, PyObject *args) {
    const char *sect;
    const char *var = "";
    int num = 1;
    if(!PyArg_ParseTuple(args, "s|s:findall", &sect, &var)) return NULL;

    IniFile ini(self->inifile);
    if(!ini) {
        PyErr_Format(error, "Internal: ini-file could not be reopened");
        return NULL;
    }

    PyObject *result = PyList_New(0);
    if(!result)
        return NULL;

    while(auto const &out = ini.findString(num, var, sect)) {
        PyList_Append(result, PyUnicode_FromString(out.value().c_str()));
        num++;
    }
    return result;
}

//
// PyTuple(filename, lineno) linuxcnc.ini.lineof(string:section, string:variable [, int:num])
//
// Return a (filname,linenr) tuple containing the filename and line number of
// the [section]variable. Optionally you can request the num'th instance of the
// variable.
//
static PyObject *Ini_lineof(pyIniFile *self, PyObject *args) {
    const char *sect;
    const char *var = "";
    int num = 1;
    if(!PyArg_ParseTuple(args, "ss|i:lineof", &sect, &var, &num)) return NULL;

    if(num < 1) {
        PyErr_Format(error, "Argument 'num' must be >= 1");
        return NULL;
    }

    IniFile ini(self->inifile);
    if(!ini) {
        PyErr_Format(error, "Internal: ini-file could not be reopened");
        return NULL;
    }

    PyObject *result = PyTuple_New(2);
    if(!result)
        return NULL;

    auto v = ini.lineOf(num, var, sect);
    if(v.second < 0) {
        // Set to (None, None)
        PyTuple_SET_ITEM(result, 0, Py_None);
        PyTuple_SET_ITEM(result, 1, Py_None);
    } else {
        PyObject *fname = PyUnicode_FromString(v.first.c_str());
        if(!fname) {
            Py_DECREF(result);
            return NULL;
        }
        PyObject *lineno = PyLong_FromLong(v.second);
        if(!lineno) {
            Py_DECREF(fname);
            Py_DECREF(result);
            return NULL;
        }
        PyTuple_SET_ITEM(result, 0, fname);
        PyTuple_SET_ITEM(result, 1, lineno);
    }
    return result;
}

//
// PyFloat|None linuxcnc.ini.maplinearunits(string:enumstr [, fallback=val])
//
// Use argument enumstr and convert the enumeration to its numerical value.
// The optional named argument fallback= defines the value to return when the
// variable is not found or invalid. The optional named option num= selects the
// num'th variable of the section (default to 1).
//
static PyObject *Ini_map_linearunits(pyIniFile * /*self*/, PyObject *args, PyObject *kwargs)
{
    const char *str;
    PyObject *def = Py_None;
    if(!PyArg_ParseTupleAndKeywords(args, kwargs, "s|$O:maplinearunits", kw_efn, &str, &def))
        return NULL;

    if(auto v = IniFile::mapLinearUnits(str))
        return PyFloat_FromDouble(*v);

    // Not found, a fallback set by argument or as None
    Py_INCREF(def);
    return def;
}

//
// PyFloat|None linuxcnc.ini.mapangularunits(string:enumstr [, fallback=val])
//
// Use argument enumstr and convert the enumeration to its numerical value.
// The optional named argument fallback= defines the value to return when the
// variable is not found or invalid. The optional named option num= selects the
// num'th variable of the section (default to 1).
//
static PyObject *Ini_map_angularunits(pyIniFile * /*self*/, PyObject *args, PyObject *kwargs)
{
    const char *str;
    PyObject *def = Py_None;
    if(!PyArg_ParseTupleAndKeywords(args, kwargs, "s|$O:mapangularunits", kw_efn, &str, &def))
        return NULL;

    if(auto v = IniFile::mapAngularUnits(str))
        return PyFloat_FromDouble(*v);

    // Not found, a fallback set by argument or as None
    Py_INCREF(def);
    return def;
}

//
// PyFloat|None linuxcnc.ini.mapjointtype(string:enumstr [, fallback=val])
//
// Use argument enumstr and convert the enumeration to its numerical value.
// The optional named argument fallback= defines the value to return when the
// variable is not found or invalid. The optional named option num= selects the
// num'th variable of the section (default to 1).
//
static PyObject *Ini_map_jointtype(pyIniFile * /*self*/, PyObject *args, PyObject *kwargs)
{
    const char *str;
    PyObject *def = Py_None;
    if(!PyArg_ParseTupleAndKeywords(args, kwargs, "s|$O:mapjointtype", kw_efn, &str, &def))
        return NULL;

    if(auto v = IniFile::mapJointType(str))
        return PyLong_FromLong((long)*v);

    // Not found, a fallback set by argument or as None
    Py_INCREF(def);
    return def;
}

//
// PyFloat|None linuxcnc.ini.getlinearunits(string:section, string:variable [, int:num] [, fallback=val])
//
// Find [section]variable and convert the enumeration to its numerical value.
// The optional named argument fallback= defines the value to return when the
// variable is not found or invalid. The optional named option num= selects the
// num'th variable of the section (default to 1).
//
static PyObject *Ini_get_linearunits(pyIniFile *self, PyObject *args, PyObject *kwargs)
{
    const char *sect, *var;
    int num = 1;
    PyObject *def = Py_None;
    if(!PyArg_ParseTupleAndKeywords(args, kwargs, "ss|i$O:getlinearunits", kw_eeefn, &sect, &var, &num, &def))
        return NULL;

    if(num < 1) {
        PyErr_Format(error, "Argument 'num' must be >= 1");
        return NULL;
    }

    IniFile ini(self->inifile);
    if(!ini) {
        PyErr_Format(error, "Internal: ini-file could not be reopened");
        return NULL;
    }

    if(auto v = ini.findLinearUnits(num, var, sect))
        return PyFloat_FromDouble(*v);

    // Not found or error
    // We have a fallback set by argument or as None
    Py_INCREF(def);
    return def;
}

//
// PyFloat|None linuxcnc.ini.getangularunits(string:section, string:variable [, int:num] [, fallback=val])
//
// Find [section]variable and convert the enumeration to its numerical value.
// The optional named argument fallback= defines the value to return when the
// variable is not found or invalid. The optional named option num= selects the
// num'th variable of the section (default to 1).
//
static PyObject *Ini_get_angularunits(pyIniFile *self, PyObject *args, PyObject *kwargs)
{
    const char *sect, *var;
    int num = 1;
    PyObject *def = Py_None;
    if(!PyArg_ParseTupleAndKeywords(args, kwargs, "ss|i$O:getangularunits", kw_eeefn, &sect, &var, &num, &def))
        return NULL;

    if(num < 1) {
        PyErr_Format(error, "Argument 'num' must be >= 1");
        return NULL;
    }

    IniFile ini(self->inifile);
    if(!ini) {
        PyErr_Format(error, "Internal: ini-file could not be reopened");
        return NULL;
    }

    if(auto v = ini.findAngularUnits(num, var, sect))
        return PyFloat_FromDouble(*v);

    // Not found or error
    // We have a fallback set by argument or as None
    Py_INCREF(def);
    return def;
}

//
// PyFloat|None linuxcnc.ini.getjointtype(string:section, string:variable [, int:num] [, fallback=val])
//
// Find [section]variable and convert the enumeration to its numerical value.
// The optional named argument fallback= defines the value to return when the
// variable is not found or invalid. The optional named option num= selects the
// num'th variable of the section (default to 1).
//
static PyObject *Ini_get_jointtype(pyIniFile *self, PyObject *args, PyObject *kwargs)
{
    const char *sect, *var;
    int num = 1;
    PyObject *def = Py_None;
    if(!PyArg_ParseTupleAndKeywords(args, kwargs, "ss|i$O:getjointtype", kw_eeefn, &sect, &var, &num, &def))
        return NULL;

    if(num < 1) {
        PyErr_Format(error, "Argument 'num' must be >= 1");
        return NULL;
    }

    IniFile ini(self->inifile);
    if(!ini) {
        PyErr_Format(error, "Internal: ini-file could not be reopened");
        return NULL;
    }

    if(auto v = ini.findJointType(num, var, sect))
        return PyLong_FromLong((long)*v);

    // Not found or error
    // We have a fallback set by argument or as None
    Py_INCREF(def);
    return def;
}

static void Ini_dealloc(pyIniFile *self) {
    if(self->inifile) {
        delete self->inifile;
        self->inifile = NULL;
    }
    PyObject_Del(self);
}

//
// This #pragma sucks...
// In C++ casting PyCFunctionWithKeywords to PyCFunction results in a warning
// about 'cast between incompatible function types'. However, this is the way
// it is done interfacing to the C API of Python.
// We disable the diagnostic warning temporarily here so we are not bothered.
// The alternative, creating an overlapping PyMethodDef structure with a
// unionized ml_meth field with all possible function signatures, is possible
// but not Python version secure.
//
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-function-type"
static PyMethodDef Ini_methods[] = {
    {"hassection", (PyCFunction)Ini_has_section, METH_VARARGS,
        "PyBool hassection(section)\n"
        "Returns a boolean indicating whether or not the given section was found "
        "in the ini-file." },
    {"hasvariable", (PyCFunction)Ini_has_variable, METH_VARARGS,
        "PyBool hasvariable(section, variable [, num])\n"
        "Returns a boolean indicating whether or not the given [section]variable "
        "was found in the ini-file. The first occurrence of the variable name will "
        "be searched if the section name is empty. The optional num argument may "
        "be used to test whether the num'th variable of that name exists in the "
        "section." },
    {"getbool", (PyCFunction)Ini_get_bool, METH_VARARGS|METH_KEYWORDS,
        "PyBool|None getbool(section, variable [, num] [, fallback=])\n"
        "Returns the value of the variable converted to boolean if it was a valid "
        "boolean. The return value is None if the variable was not found or an "
        "invalid boolean value was detected and no fallback= was provided. The "
        "optional num argument may be used to select the num'th variable of that "
        "name in the section." },
    {"getsint", (PyCFunction)Ini_get_sint, METH_VARARGS|METH_KEYWORDS,
        "PyInt|None getsint(section, variable [, num] [, fallback=])\n"
        "Returns the value of the variable converted to signed integer if it was a "
        "valid integer. The return value is None if the variable was not found or "
        "an invalid value was detected and no fallback= was provided. The optional "
        "num argument may be used to select the num'th variable of that name in "
        "the section." },
    {"getint", (PyCFunction)Ini_get_sint, METH_VARARGS|METH_KEYWORDS,
        "PyInt|None getint(section, variable [, num] [, fallback=])\n"
        "Alias of getsint" },
    {"getuint", (PyCFunction)Ini_get_uint, METH_VARARGS|METH_KEYWORDS,
        "PyInt|None getuint(section, variable [, num] [, fallback=])\n"
        "Returns the value of the variable converted to unsigned integer if it was "
        "a valid unsigned integer. The return value is None if the variable was "
        "not found or an invalid value was detected and no fallback= was provided. "
        "The optional num argument may be used to select the num'th variable of "
        "that name in the section." },
    {"getreal", (PyCFunction)Ini_get_real, METH_VARARGS|METH_KEYWORDS,
        "PyFloat|None getreal(section, variable [, num] [, fallback=])\n"
        "Returns the value of the variable converted to floating point real if it "
        "was a valid real. The return value is None if the variable was not found "
        "or an invalid value was detected and no fallback= was provided. The "
        "optional num argument may be used to select the num'th variable of that "
        "name in the section." },
    {"getfloat", (PyCFunction)Ini_get_real, METH_VARARGS|METH_KEYWORDS,
        "PyFloat|None getfloat(section, variable [, num] [, fallback=])\n"
        "Alias of getreal()." },
    {"getstring", (PyCFunction)Ini_get_string, METH_VARARGS|METH_KEYWORDS,
        "PyString|None getstring(section, variable [, num] [, fallback=])\n"
        "Returns the value of the variable as a string if it exists. "
        "The return value is None if the variable was not found and no fallback= "
        "was provided. The optional num argument may be used to select the num'th "
        "variable of that name in the section." },
    {"getsections", (PyCFunction)Ini_get_sections, METH_VARARGS,
        "PyList getsections()\n"
        "Returns a list of section names. " },
    {"getvariables", (PyCFunction)Ini_get_variables, METH_VARARGS,
        "PyList(PyTuple(name,value)) getvariables([section])\n"
        "Returns a list of (name,value) tuples of all variables in the named "
        "section or the variables from all sections if the section name is not "
        "specified." },
    {"find", (PyCFunction)Ini_get_string, METH_VARARGS|METH_KEYWORDS,
        "PyString|None find(section, variable [, num] [, fallback=])\n"
        "Alias of getstring()" },
    {"findall", (PyCFunction)Ini_findall, METH_VARARGS,
        "PyList findall(section [,variable])\n"
        "Find value(s) from named section in inifile as a list matching the "
        "variable name." },
    {"lineof", (PyCFunction)Ini_lineof, METH_VARARGS,
        "PyTuple(filename,lineno) lineof(section, variable [, num])\n"
        "Returns a tuple with the filename and line number of the num'th "
        "variable in the section. The first matching section variable is "
        "returned if num if not provided. The tuple (None, None) is returned "
        "if the variable is not found." },
    {"maplinearunits", (PyCFunction)Ini_map_linearunits, METH_VARARGS|METH_KEYWORDS,
        "PyFloat|None maplinearunits(enumstr [, fallback=])\n"
        "Take the enumeration string argument and try to convert. "
        "Returns the value associated with enumerated type defined by "
        "[mm, metric, in, inch, imperial]." },
    {"mapangularunits", (PyCFunction)Ini_map_angularunits, METH_VARARGS|METH_KEYWORDS,
        "PyFloat|None mapangularunits(enumstr [, fallback=])\n"
        "Take the enumeration string argument and try to convert. "
        "Returns the value associated with enumerated type defined by "
        "[deg, degree, grad, gon, rad, radian]." },
    {"mapjointtype", (PyCFunction)Ini_map_jointtype, METH_VARARGS|METH_KEYWORDS,
        "PyInt|None mapjointtype(enumstr [, fallback=])\n"
        "Take the enumeration string argument and try to convert. "
        "Returns the value associated with enumerated type defined by "
        "[LINEAR, ANGULAR]." },
    {"getlinearunits", (PyCFunction)Ini_get_linearunits, METH_VARARGS|METH_KEYWORDS,
        "PyFloat|None getlinearunits(section, variable [, num] [, fallback=])\n"
        "Get the ini variable from the section and convert the enumerated type. "
        "The optional num argument may be used to select the num'th variable of "
        "that name in the section. Returns the value associated with enumerated "
        "type defined by [mm, metric, in, inch, imperial]." },
    {"getangularunits", (PyCFunction)Ini_get_angularunits, METH_VARARGS|METH_KEYWORDS,
        "PyFloat|None getangularunits(section, variable [, num] [, fallback=])\n"
        "Get the ini variable from the section and convert the enumerated type. "
        "The optional num argument may be used to select the num'th variable of "
        "that name in the section. Returns the value associated with enumerated "
        "type defined by [deg, degree, grad, gon, rad, radian]." },
    {"getjointtype", (PyCFunction)Ini_get_jointtype, METH_VARARGS|METH_KEYWORDS,
        "PyInt|None getjointtype(section, variable [, num] [, fallback=])\n"
        "Get the ini variable from the section and convert the enumerated type. "
        "The optional num argument may be used to select the num'th variable of "
        "that name in the section. Returns the value associated with enumerated "
        "type defined by [LINEAR, ANGULAR]." },
    {}
};
#pragma GCC diagnostic pop

static const char linuxcncinidoc[] =
    "LinuxCNC INI-file reader, parser and query module.\n"
    "Instantiate with:\n"
    "  cfg = linuxcnc.ini('path_to_ini_file.ini')\n"
    "\n"
    "Available methods:\n"
    "  PyBool hassection(section)\n"
    "  PyBool hasvariable(section, variable)\n"
    "  PyBool|None getbool(section, variable [, num] [, fallback=])\n"
    "  PyInt|None getsint(section, variable [, num] [, fallback=])\n"
    "  PyInt|None getuint(section, variable [, num] [, fallback=])\n"
    "  PyFloat|None getreal(section, variable [, num] [, fallback=])\n"
    "  PyString|None getstring(section, variable [, num] [, fallback=])\n"
    "  PyList getsections()\n"
    "  PyList(PyTuple(name,value)) getvariables([section])\n"
    "  PyList findall(section [,variable])\n"
    "  PyTuple(filename,lineno) lineof(section, variable [, num])\n"
    "  PyFloat|None getlinearunits(section, variable [, num] [, fallback=])\n"
    "  PyFloat|None getangularunits(section, variable [, num] [, fallback=])\n"
    "  PyInt|None getjointtype(section, variable [, num] [, fallback=])\n"
    "  PyFloat|None maplinearunits(enumstr [, fallback=])\n"
    "  PyFloat|None mapangularunits(enumstr [, fallback=])\n"
    "  PyInt|None mapjointtype(enumstr [, fallback=])\n"
    "\n"
    "Several convenience methods are provided as aliases to above methods:\n"
    "  PyInt|None getint(section, variable [, num] [, fallback=])\n"
    "  PyFloat|None getfloat(section, variable [, num] [, fallback=])\n"
    "  PyString|None find(section, variable [, num] [, fallback=])\n"
    "\n"
    "Method documentation is provided with each method.\n"
    ;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wzero-as-null-pointer-constant"
static PyTypeObject Ini_Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "linuxcnc.ini",              /*tp_name*/
    sizeof(pyIniFile),      /*tp_basicsize*/
    0,                      /*tp_itemsize*/
    /* methods */
    (destructor)Ini_dealloc,/*tp_dealloc*/
    0,                      /*tp_print*/
    0,                      /*tp_getattr*/
    0,                      /*tp_setattr*/
    0,                      /*tp_compare*/
    0,                      /*tp_repr*/
    0,                      /*tp_as_number*/
    0,                      /*tp_as_sequence*/
    0,                      /*tp_as_mapping*/
    0,                      /*tp_hash*/
    0,                      /*tp_call*/
    0,                      /*tp_str*/
    0,                      /*tp_getattro*/
    0,                      /*tp_setattro*/
    0,                      /*tp_as_buffer*/
    Py_TPFLAGS_DEFAULT,     /*tp_flags*/
    linuxcncinidoc,         /*tp_doc*/
    0,                      /*tp_traverse*/
    0,                      /*tp_clear*/
    0,                      /*tp_richcompare*/
    0,                      /*tp_weaklistoffset*/
    0,                      /*tp_iter*/
    0,                      /*tp_iternext*/
    Ini_methods,            /*tp_methods*/
    0,                      /*tp_members*/
    0,                      /*tp_getset*/
    0,                      /*tp_base*/
    0,                      /*tp_dict*/
    0,                      /*tp_descr_get*/
    0,                      /*tp_descr_set*/
    0,                      /*tp_dictoffset*/
    (initproc)Ini_init,     /*tp_init*/
    0,                      /*tp_alloc*/
    PyType_GenericNew,      /*tp_new*/
    0,                      /*tp_free*/
    0,                      /*tp_is_gc*/
    0,                      /*tp_bases*/
    0,                      /*tp_mro*/
    0,                      /*tp_cache*/
    0,                      /*tp_subclasses*/
    0,                      /*tp_weaklink*/
    0,                      /*tp_del*/
    0,                      /*tp_version_tag*/
    0,                      /*tp_finalize*/
#if PY_VERSION_HEX >= 0x030800f0	// 3.8
    0,                      /*tp_vectorcall*/
#if PY_VERSION_HEX >= 0x030c00f0	// 3.12
    0,                      /*tp_watched*/
#if PY_VERSION_HEX >= 0x030d00f0	// 3.13
    0,                      /*tp_versions_used*/
#endif
#endif
#endif
};
#pragma GCC diagnostic pop


static PyStructSequence_Field tool_fields[] = {
    {(char*)"id", NULL},
    {(char*)"xoffset", NULL},
    {(char*)"yoffset", NULL},
    {(char*)"zoffset", NULL},
    {(char*)"aoffset", NULL},
    {(char*)"boffset", NULL},
    {(char*)"coffset", NULL},
    {(char*)"uoffset", NULL},
    {(char*)"voffset", NULL},
    {(char*)"woffset", NULL},
    {(char*)"diameter", NULL},
    {(char*)"frontangle", NULL},
    {(char*)"backangle", NULL},
    {(char*)"orientation", NULL},
    {},
};

static PyStructSequence_Desc tool_result_desc = {
    (char*)"tool_result", /* name */
    (char*)"", /* doc */
    tool_fields,
    14
};

static PyTypeObject ToolResultType;


#define AXIS_MASK_A 0x08
#define AXIS_MASK_B 0x10
#define AXIS_MASK_C 0x20
static struct rotation_offsets {
    double x;
    double y;
    double z;
    unsigned int axis_mask;
    unsigned int respect_offsets;
} roffsets;

static void rotate_z(double pt[3], double a) {
    double theta = a * M_PI / 180;
    double c = cos(theta), s = sin(theta);
    double tx, ty;
    if (roffsets.respect_offsets) {
        tx = (pt[0]-roffsets.x) * c - (pt[1]-roffsets.y) * s;
        ty = (pt[0]-roffsets.x) * s + (pt[1]-roffsets.y) * c;
    } else {
        tx = pt[0] * c - pt[1] * s;
        ty = pt[0] * s + pt[1] * c;
    }

    pt[0] = tx; pt[1] = ty;
}

static void rotate_y(double pt[3], double a) {
    double theta = a * M_PI / 180;
    double c = cos(theta), s = sin(theta);
    double tx, tz;
    if (roffsets.respect_offsets) {
        tx = (pt[0]-roffsets.x) * c - (pt[2]-roffsets.z) * s;
        tz = (pt[0]-roffsets.x) * s + (pt[2]-roffsets.z) * c;
    } else {
        tx = pt[0] * c - pt[2] * s;
        tz = pt[0] * s + pt[2] * c;
    }

    pt[0] = tx; pt[2] = tz;
}

static void rotate_x(double pt[3], double a) {
    double theta = a * M_PI / 180;
    double c = cos(theta), s = sin(theta);
    double ty, tz;
    if (roffsets.respect_offsets) {
        ty = (pt[1]-roffsets.y) * c - (pt[2]-roffsets.z) * s;
        tz = (pt[1]-roffsets.y) * s + (pt[2]-roffsets.z) * c;
    } else {
        ty = pt[1] * c - pt[2] * s;
        tz = pt[1] * s + pt[2] * c;
    }

    pt[1] = ty; pt[2] = tz;
}

static void translate(double pt[3], double ox, double oy, double oz) {
    pt[0] += ox;
    pt[1] += oy;
    pt[2] += oz;
}

static void vertex9(const double pt[9], double p[3], const char *geometry) {
    double sign = 1;

    p[0] = 0;
    p[1] = 0;
    p[2] = 0;

    for(; *geometry; geometry++) {
        switch(*geometry) {
            case '-': sign = -1; break;
            case 'X': translate(p, pt[0] * sign, 0, 0); sign=1; break;
            case 'Y': translate(p, 0, pt[1] * sign, 0); sign=1; break;
            case 'Z': translate(p, 0, 0, pt[2] * sign); sign=1; break;
            case 'U': translate(p, pt[6] * sign, 0, 0); sign=1; break;
            case 'V': translate(p, 0, pt[7] * sign, 0); sign=1; break;
            case 'W': translate(p, 0, 0, pt[8] * sign); sign=1; break;
            case 'A': if (roffsets.axis_mask & AXIS_MASK_A) {
                          rotate_x(p, pt[3] * sign);
                      }
                      sign=1; break;
            case 'B': if (roffsets.axis_mask & AXIS_MASK_B) {
                          rotate_y(p, pt[4] * sign);
                      }
                      sign=1; break;
            case 'C': if (roffsets.axis_mask & AXIS_MASK_C) {
                          rotate_z(p, pt[5] * sign);
                      }
                      sign=1; break;

        }
    }
}

// Retired: this emitted immediate-mode OpenGL vertices, which the 3.3 core
// profile used for preview rendering does not have.  The name, signature,
// argument checking, and return value are kept for out-of-tree callers; it no
// longer draws.  Replacement: rs274.glcanon_scene.
static PyObject *pyline9(PyObject * /*s*/, PyObject *o) {
    static bool warned = false;
    double pt1[9], pt2[9];
    const char *geometry;

    if(!PyArg_ParseTuple(o, "s(ddddddddd)(ddddddddd):line9",
            &geometry,
            &pt1[0], &pt1[1], &pt1[2],
            &pt1[3], &pt1[4], &pt1[5],
            &pt1[6], &pt1[7], &pt1[8],
            &pt2[0], &pt2[1], &pt2[2],
            &pt2[3], &pt2[4], &pt2[5],
            &pt2[6], &pt2[7], &pt2[8]))
        return NULL;

    if(!warned) {
        warned = true;
        if(PyErr_WarnEx(PyExc_DeprecationWarning,
                    "linuxcnc.line9() no longer draws; use rs274.glcanon_scene",
                    1) < 0)
            return NULL;
    }

    Py_RETURN_NONE;
}

static PyObject *pyvertex9(PyObject * /*s*/, PyObject *o) {
    double pt1[9], pt[3];
    char *geometry;
    if(!PyArg_ParseTuple(o, "s(ddddddddd):vertex9",
            &geometry,
            &pt1[0], &pt1[1], &pt1[2],
            &pt1[3], &pt1[4], &pt1[5],
            &pt1[6], &pt1[7], &pt1[8]))
        return NULL;

    vertex9(pt1, pt, geometry);
    return Py_BuildValue("(ddd)", pt[0], pt[1], pt[2]);
}

static PyObject *pygui_respect_offsets (PyObject * /*s*/, PyObject *o) {
    char* coords;

    if(!PyArg_ParseTuple(o, "si",&coords, &roffsets.respect_offsets)) {
        return NULL;
    }
    if (roffsets.respect_offsets) {
        // GEOMETRY rotations only if letters (ABC) included in [TRAJ]COORDINATES
        if (strchr(coords,'A')) roffsets.axis_mask |= AXIS_MASK_A;
        if (strchr(coords,'B')) roffsets.axis_mask |= AXIS_MASK_B;
        if (strchr(coords,'C')) roffsets.axis_mask |= AXIS_MASK_C;
    }

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *pygui_rot_offsets(PyObject * /*s*/, PyObject *o) {
    if(!PyArg_ParseTuple(o, "ddd", &roffsets.x,&roffsets.y,&roffsets.z)) {
        return NULL;
    }
    Py_INCREF(Py_None);
    return Py_None;
}

// Retired: this drew with immediate-mode OpenGL (glBegin/glVertex*/glEnd),
// invalid in the 3.3 core profile now used for preview rendering.  The name,
// signature, argument checking, and return value are kept for out-of-tree
// callers; it no longer draws.  Replacement: rs274.glcanon_scene.
static PyObject *pydraw_lines(PyObject * /*s*/, PyObject *o) {
    static bool warned = false;
    PyListObject *li;
    int for_selection = 0;
    int i;
    int n;
    double p1[9], p2[9];
    char *geometry;

    if(!PyArg_ParseTuple(o, "sO!|i:draw_lines",
			    &geometry, &PyList_Type, &li, &for_selection))
        return NULL;

    for(i=0; i<PyList_GET_SIZE(li); i++) {
        PyObject *it = PyList_GET_ITEM(li, i);
        PyObject *dummy1, *dummy2, *dummy3;
        if(!PyArg_ParseTuple(it, "i(ddddddddd)(ddddddddd)|OOO", &n,
                    p1+0, p1+1, p1+2,
                    p1+3, p1+4, p1+5,
                    p1+6, p1+7, p1+8,
                    p2+0, p2+1, p2+2,
                    p2+3, p2+4, p2+5,
                    p2+6, p2+7, p2+8,
                    &dummy1, &dummy2, &dummy3))
            return NULL;
    }

    if(!warned) {
        warned = true;
        if(PyErr_WarnEx(PyExc_DeprecationWarning,
                    "linuxcnc.draw_lines() no longer draws; use rs274.glcanon_scene",
                    1) < 0)
            return NULL;
    }

    Py_INCREF(Py_None);
    return Py_None;
}

// Retired: this drew with immediate-mode OpenGL (glBegin/glVertex*/glEnd),
// invalid in the 3.3 core profile now used for preview rendering.  The name,
// signature, argument checking, and return value are kept for out-of-tree
// callers; it no longer draws.  Replacement: rs274.glcanon_scene.
static PyObject *pydraw_dwells(PyObject * /*s*/, PyObject *o) {
    static bool warned = false;
    PyListObject *li;
    int for_selection = 0, is_lathe = 0, i, n;
    double alpha;
    char *geometry;

    if(!PyArg_ParseTuple(o, "sO!dii:draw_dwells", &geometry, &PyList_Type, &li, &alpha, &for_selection, &is_lathe))
        return NULL;

    for(i=0; i<PyList_GET_SIZE(li); i++) {
        PyObject *it = PyList_GET_ITEM(li, i);
        double red, green, blue, x, y, z;
        int axis;
        if(!PyArg_ParseTuple(it, "i(ddd)dddi", &n, &red, &green, &blue, &x, &y, &z, &axis)) {
            return NULL;
        }
    }

    if(!warned) {
        warned = true;
        if(PyErr_WarnEx(PyExc_DeprecationWarning,
                    "linuxcnc.draw_dwells() no longer draws; use rs274.glcanon_scene",
                    1) < 0)
            return NULL;
    }

    Py_INCREF(Py_None);
    return Py_None;
}



static PyMethodDef emc_methods[] = {
#define METH(name, doc) { #name, (PyCFunction) py##name, METH_VARARGS, doc }
METH(draw_lines, "Retired: no longer draws, use rs274.glcanon_scene"),
METH(draw_dwells, "Retired: no longer draws, use rs274.glcanon_scene"),
METH(line9, "Retired: no longer draws, use rs274.glcanon_scene"),
METH(vertex9, "Get the 3d location for a 9d point"),
METH(gui_rot_offsets, "Set x,y,z offsets for A,B,C rotations"),
METH(gui_respect_offsets, "Enable rotations about g5x,g92 offsets"),
    {}
#undef METH
};

/* ENUM defines an integer constant with the same name as the C constant.
 * ENUMX defines an integer constant with the first i characters of the C
 * constant name removed (so that ENUMX(4,RCS_TASK_MODE_MDI) creates a constant
 * named TASK_MODE_MDI) */

#define ENUM(e) PyModule_AddIntConstant(m, const_cast<char*>(#e), e)
#define ENUMX(x,e) PyModule_AddIntConstant(m, x + const_cast<char*>(#e), e)

static struct PyModuleDef linuxcnc_moduledef = {
    PyModuleDef_HEAD_INIT, /* m_base */
    "linuxcnc",            /* m_name */
    "LinuxCNC INI reader and constants (no NML: the bus is NATS)", /* m_doc */
    -1,                    /* m_size */
    emc_methods,           /* m_methods */
    NULL,                  /* m_slots */
    NULL,                  /* m_travese */
    NULL,                  /* m_clear */
    NULL,                  /* m_free */
};

PyMODINIT_FUNC PyInit_linuxcnc(void);
PyMODINIT_FUNC PyInit_linuxcnc(void)
{

    PyType_Ready(&Ini_Type);
    error = PyErr_NewException((char*)"linuxcnc.error", PyExc_RuntimeError, NULL);

    m = PyModule_Create(&linuxcnc_moduledef);
    PyModule_AddObject(m, "ini", (PyObject*)&Ini_Type);
    PyModule_AddObject(m, "error", error);


    PyModule_AddStringConstant(m, "PREFIX", EMC2_HOME);
    PyModule_AddStringConstant(m, "SHARE", EMC2_HOME "/share");

    PyModule_AddIntConstant(m, "OPERATOR_ERROR", EMC_OPERATOR_ERROR_TYPE);
    PyModule_AddIntConstant(m, "OPERATOR_TEXT", EMC_OPERATOR_TEXT_TYPE);
    PyModule_AddIntConstant(m, "OPERATOR_DISPLAY", EMC_OPERATOR_DISPLAY_TYPE);

    PyStructSequence_InitType(&ToolResultType, &tool_result_desc);
    PyModule_AddObject(m, "tool", (PyObject*)&ToolResultType);
    PyModule_AddObject(m, "version", PyUnicode_FromString(PACKAGE_VERSION));

    ENUMX(4, EMC_LINEAR);
    ENUMX(4, EMC_ANGULAR);

    PyModule_AddIntConstant(m, "INTERP_IDLE", (int)EMC_TASK_INTERP::IDLE);
    PyModule_AddIntConstant(m, "INTERP_READING", (int)EMC_TASK_INTERP::READING);
    PyModule_AddIntConstant(m, "INTERP_PAUSED", (int)EMC_TASK_INTERP::PAUSED);
    PyModule_AddIntConstant(m, "INTERP_WAITING", (int)EMC_TASK_INTERP::WAITING);

    PyModule_AddIntConstant(m, "MODE_MDI", (int)EMC_TASK_MODE::MDI);
    PyModule_AddIntConstant(m, "MODE_MANUAL", (int)EMC_TASK_MODE::MANUAL);
    PyModule_AddIntConstant(m, "MODE_AUTO", (int)EMC_TASK_MODE::AUTO);

    PyModule_AddIntConstant(m, "STATE_OFF", (int)EMC_TASK_STATE::OFF);
    PyModule_AddIntConstant(m, "STATE_ON", (int)EMC_TASK_STATE::ON);
    PyModule_AddIntConstant(m, "STATE_ESTOP", (int)EMC_TASK_STATE::ESTOP);
    PyModule_AddIntConstant(m, "STATE_ESTOP_RESET", (int)EMC_TASK_STATE::ESTOP_RESET);

    ENUMX(6, LOCAL_SPINDLE_FORWARD);
    ENUMX(6, LOCAL_SPINDLE_REVERSE);
    ENUMX(6, LOCAL_SPINDLE_OFF);
    ENUMX(6, LOCAL_SPINDLE_INCREASE);
    ENUMX(6, LOCAL_SPINDLE_DECREASE);
    ENUMX(6, LOCAL_SPINDLE_CONSTANT);

    ENUMX(6, LOCAL_MIST_ON);
    ENUMX(6, LOCAL_MIST_OFF);

    ENUMX(6, LOCAL_FLOOD_ON);
    ENUMX(6, LOCAL_FLOOD_OFF);

    ENUMX(6, LOCAL_BRAKE_ENGAGE);
    ENUMX(6, LOCAL_BRAKE_RELEASE);

    ENUMX(6, LOCAL_JOG_STOP);
    ENUMX(6, LOCAL_JOG_CONTINUOUS);
    ENUMX(6, LOCAL_JOG_INCREMENT);

    ENUMX(6, LOCAL_AUTO_RUN);
    ENUMX(6, LOCAL_AUTO_PAUSE);
    ENUMX(6, LOCAL_AUTO_RESUME);
    ENUMX(6, LOCAL_AUTO_STEP);
    ENUMX(6, LOCAL_AUTO_REVERSE);
    ENUMX(6, LOCAL_AUTO_FORWARD);

    PyModule_AddIntConstant(m, "TRAJ_MODE_FREE", (int)EMC_TRAJ_MODE::FREE);
    PyModule_AddIntConstant(m, "TRAJ_MODE_COORD", (int)EMC_TRAJ_MODE::COORD);
    PyModule_AddIntConstant(m, "TRAJ_MODE_TELEOP", (int)EMC_TRAJ_MODE::TELEOP);

    ENUMX(4, EMC_MOTION_TYPE_TRAVERSE);
    ENUMX(4, EMC_MOTION_TYPE_FEED);
    ENUMX(4, EMC_MOTION_TYPE_ARC);
    ENUMX(4, EMC_MOTION_TYPE_TOOLCHANGE);
    ENUMX(4, EMC_MOTION_TYPE_PROBING);
    ENUMX(4, EMC_MOTION_TYPE_INDEXROTARY);

    ENUM(KINEMATICS_IDENTITY);
    ENUM(KINEMATICS_FORWARD_ONLY);
    ENUM(KINEMATICS_INVERSE_ONLY);
    ENUM(KINEMATICS_BOTH);

    ENUMX(4, EMC_DEBUG_CONFIG);
    ENUMX(4, EMC_DEBUG_VERSIONS);
    ENUMX(4, EMC_DEBUG_TASK_ISSUE);
    ENUMX(4, EMC_DEBUG_NML);
    ENUMX(4, EMC_DEBUG_MOTION_TIME);
    ENUMX(4, EMC_DEBUG_INTERP);
    ENUMX(4, EMC_DEBUG_RCS);
    ENUMX(4, EMC_DEBUG_INTERP_LIST);
    ENUMX(4, EMC_DEBUG_OWORD);
    ENUMX(4, EMC_DEBUG_REMAP);
    ENUMX(4, EMC_DEBUG_PYTHON);
    ENUMX(4, EMC_DEBUG_STATE_TAGS);

    PyModule_AddIntConstant(m, "EXEC_ERROR", (int)EMC_TASK_EXEC::ERROR);
    PyModule_AddIntConstant(m, "EXEC_DONE", (int)EMC_TASK_EXEC::DONE);
    PyModule_AddIntConstant(m, "EXEC_WAITING_FOR_MOTION", (int)EMC_TASK_EXEC::WAITING_FOR_MOTION);
    PyModule_AddIntConstant(m, "EXEC_WAITING_FOR_MOTION_QUEUE", (int)EMC_TASK_EXEC::WAITING_FOR_MOTION_QUEUE);
    PyModule_AddIntConstant(m, "EXEC_WAITING_FOR_IO", (int)EMC_TASK_EXEC::WAITING_FOR_IO);
    PyModule_AddIntConstant(m, "EXEC_WAITING_FOR_MOTION_AND_IO", (int)EMC_TASK_EXEC::WAITING_FOR_MOTION_AND_IO);
    PyModule_AddIntConstant(m, "EXEC_WAITING_FOR_DELAY", (int)EMC_TASK_EXEC::WAITING_FOR_DELAY);
    PyModule_AddIntConstant(m, "EXEC_WAITING_FOR_SYSTEM_CMD", (int)EMC_TASK_EXEC::WAITING_FOR_SYSTEM_CMD);
    PyModule_AddIntConstant(m, "EXEC_WAITING_FOR_SPINDLE_ORIENTED", (int)EMC_TASK_EXEC::WAITING_FOR_SPINDLE_ORIENTED);

    ENUMX(7, EMCMOT_MAX_JOINTS);
    ENUMX(7, EMCMOT_MAX_AXIS);


    PyModule_AddIntConstant(m, "RCS_DONE", (int)RCS_STATUS::DONE);
    PyModule_AddIntConstant(m, "RCS_EXEC", (int)RCS_STATUS::EXEC);
    PyModule_AddIntConstant(m, "RCS_ERROR", (int)RCS_STATUS::ERROR);
    return m;
}


// # vim:sw=4:sts=4:et:ts=8:
