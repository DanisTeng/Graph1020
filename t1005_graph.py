from typing import List, Tuple, Dict, Set
import sympy as sp
from sympy.printing.cxx import CXX11CodePrinter

_lower_alphabet_chars = [chr(i) for i in range(97, 123)]
_upper_alphabet_chars = [chr(i) for i in range(65, 91)]
_number_chars = [chr(i) for i in range(48, 58)]

_sympy_var_prefix = 'sympyvar'
_unnamed_graph_var_prefix = 'graph_var_'
_graph_output_prefix = 'output_'
_node_derivative_prefix = 'NODE_'
_graph_derivative_prefix = "GRAPH_"
_built_in_name_sub_strings = [_sympy_var_prefix, _unnamed_graph_var_prefix, _graph_output_prefix]

_indent = "  "


def _decorate_function_name_with_derivatives(function_name, add_first_order_derivative=False,
                                             add_second_order_derivative=False):
    mask = [add_first_order_derivative, add_second_order_derivative]
    titles = ['First', 'Second']
    if any(mask):
        result = function_name
        result += 'With'
        for i in range(len(mask)):
            if mask[i]:
                result += titles[i]
        result += 'OrderDerivatives'
        return result
    else:
        return function_name


def _is_valid_cpp_name(cpp_name: str):
    count = 0
    for c in cpp_name:
        if (count > 0 and c in _number_chars) or \
                c in _lower_alphabet_chars or \
                c in _upper_alphabet_chars or \
                c == '_':
            pass
        else:
            return False
        count += 1
    return count > 0


def _is_valid_namespace(namespace: str):
    return all([_is_valid_cpp_name(sub_name) for sub_name in namespace.split("::")])


# sympy optimizations

def opt_separate_exp_by2(expr: sp.Basic):
    if not isinstance(expr, (sp.Basic,)):
        return expr
    if not expr.args:
        return expr
    # ... TODO(): more optimization
    return expr


class OptimizedCXX11Printer(CXX11CodePrinter):
    # TODO(): More optimization
    pass


class Variable:
    # rear can convert into front, vise NOT versa
    # TODO(): Consider whether it is possible that one claim ArrayXd var = 1.0;
    var_types = ['ArrayXd', 'double', 'float', 'int']
    var_types_not_need_const_reference = ['double', 'int']
    default_var_type = 'double'

    TYPE_UN_DEFINED = -1
    TYPE_STATE_INPUT = 0
    TYPE_CONSTANT = 1
    TYPE_CONFIG_INPUT = 2
    TYPE_STATE_EXPR = 3
    TYPE_CONFIG_EXPR = 4
    TYPE_CONSTANT_EXPR = 5

    def __init__(self, name: str, nick_name: str, graph: "Graph"):
        self.name = name
        self.nick_name = nick_name
        self.graph = graph

        self.var_type = self.default_var_type
        self.type = self.TYPE_UN_DEFINED

        # That how we get this variable
        # variables must be from one of the following ways:
        # 1, As input
        # 2, As constant
        # 3, As function of Variables

        # We shall also use constant to control the data flow? Nope

        # how many types does a variable have? scalar?
        # The batch-ed scalar?, scalar real?

        # if you want the power of fast development, you must make it super easy to use.
        # No compiler error is tolerated.

        # it is a environment that multiple different scalar reals are interacting with each other, they are 2-2 transferable?

        # We must be able to infer the type to be 1 dim or many dims.

        # double + int -> double
        # double + float -> double
        # double + ArrayNd -> ArrayNd

        # definition

    def is_differentiable(self):
        return self.type in [self.TYPE_STATE_EXPR, self.TYPE_STATE_INPUT]

    def set_name(self, name: str):
        self.graph.re_name(self.nick_name, name)

    def defined_as_state_input(self, var_type: str = 'double'):
        assert self.type is self.TYPE_UN_DEFINED, "re definition not allowed"
        assert var_type in self.var_types, "Invalid type: %s" % var_type

        self.type = self.TYPE_STATE_INPUT
        self.var_type = var_type

    def defined_as_constant(self, value: float, var_type: str = 'double'):
        """
        :param value:
        :param var_type:
        :return:
        """
        assert self.type is self.TYPE_UN_DEFINED, "re definition not allowed"
        assert var_type in self.var_types, "Invalid type: %s" % var_type

        self.value = value
        self.type = self.TYPE_CONSTANT
        self.var_type = var_type

    def defined_as_config_input(self, var_type: str = 'double'):
        """
        :param var_type:
        :return:
        """
        assert self.type is self.TYPE_UN_DEFINED, "re definition not allowed"
        assert var_type in self.var_types, "Invalid type: %s" % var_type

        self.type = self.TYPE_CONFIG_INPUT
        self.var_type = var_type

    def defined_as_expr(self, function: "FunctionBase", inputs: List["Variable"], output_channel=0,
                        var_type: str = 'double'):
        """
        :param function:
        :param inputs:
        :param output_channel:
        :param var_type:
        :return:
        """
        assert self.type is self.TYPE_UN_DEFINED, "re definition not allowed"
        assert var_type in self.var_types, "Invalid type: %s" % var_type

        input_types = [input_variable.type for input_variable in inputs]
        assert Variable.TYPE_UN_DEFINED not in input_types
        if Variable.TYPE_STATE_EXPR in input_types or Variable.TYPE_STATE_INPUT in input_types:
            output_type = Variable.TYPE_STATE_EXPR
        elif Variable.TYPE_CONFIG_EXPR in input_types or Variable.TYPE_CONFIG_INPUT in input_types:
            output_type = Variable.TYPE_CONFIG_EXPR
        else:
            output_type = Variable.TYPE_CONSTANT_EXPR

        self.function = function
        self.inputs = inputs.copy()
        self.output_channel = output_channel
        self.type = output_type
        self.var_type = var_type

    @classmethod
    def infer_combined_type(cls, var_type_list: List[str]):
        if not var_type_list:
            return Variable.default_var_type

        var_type_indices = {cls.var_types[i]: i for i in range(len(cls.var_types))}
        min_index = len(cls.var_types) - 1
        for var_type in var_type_list:
            assert var_type in var_type_indices.keys()
            min_index = min(min_index, var_type_indices[var_type])

        return cls.var_types[min_index]

    @classmethod
    def const_reference(cls, var_type: str):
        if var_type in cls.var_types_not_need_const_reference:
            return var_type
        else:
            return "const " + var_type + "&"

    # Some basic operators
    # not recommended for massive use
    def __add__(self, other):
        return _SymPyOperatorFunctions.sympy_add(self, other)

    def __radd__(self, other):
        return _SymPyOperatorFunctions.sympy_add(other, self)

    def __sub__(self, other):
        return _SymPyOperatorFunctions.sympy_sub(self, other)

    def __rsub__(self, other):
        return _SymPyOperatorFunctions.sympy_sub(other, self)

    def __mul__(self, other):
        return _SymPyOperatorFunctions.sympy_mul(self, other)

    def __rmul__(self, other):
        return _SymPyOperatorFunctions.sympy_mul(other, self)

    def __pow__(self, other):
        return _SymPyOperatorFunctions.sympy_pow(self, other)

    def __rpow__(self, other):
        return _SymPyOperatorFunctions.sympy_pow(other, self)

    def __truediv__(self, other):
        return _SymPyOperatorFunctions.sympy_div(self, other)

    def __rtruediv__(self, other):
        return _SymPyOperatorFunctions.sympy_div(other, self)


class FunctionContext:
    """
    Function context is all what we need for a function action to be defined.

    FunctionContext determines the type, var_type and the name of the input and outputs.

    It skips those zero/trivial outputs.

    For how to call the function, it depends on the class.
    """

    def __init__(self, inputs: List[Variable], outputs: List[Variable]):
        self.input_variables = inputs
        self.output_variables = outputs

    @staticmethod
    def first_order_derivative_name(out_name, in_name):
        # We guaranteed that out_name, in_name are lower case in graph.
        return "D_%s_D_%s" % (out_name, in_name)

    @staticmethod
    def second_order_derivative_name(out_name, in_name_1, in_name_2):
        # We guaranteed that out_name, in_name are lower case in graph.
        return "D2_%s_D_%s_D_%s" % (
            out_name,
            in_name_1,
            in_name_2)

    # To provide mutual agreement on the outputs of function
    def get_first_order_derivatives_info(self):
        """
        :return: a list of tuple:
         ((output_idx: int, input_idx: int), node_derivative_name: str, node_derivative_type: str)
        """
        # assume output types are correctly given.
        # When given output var types, and input var types, is it possible to determine derivative types?
        out_dim = len(self.output_variables)
        in_dim = len(self.input_variables)
        results = []

        for i in range(out_dim):
            if not self.output_variables[i].is_differentiable():
                continue
            for j in range(in_dim):
                if not self.input_variables[j].is_differentiable():
                    continue
                node_derivative_name = \
                    _node_derivative_prefix + self.first_order_derivative_name(
                        self.output_variables[i].nick_name, self.input_variables[j].nick_name)
                node_derivative_type = self.output_variables[i].var_type

                results.append(((i, j), node_derivative_name, node_derivative_type))

        return results

    # To provide mutual agreement on the outputs of function
    def get_second_order_derivatives_info(self):
        """
        :return: a list of tuple:
         ((output_idx: int, input_idx: int, input_idx: int), node_derivative_name: str, node_derivative_type: str)
        """

        # assume output types are correctly given.
        # When given output var types, and input var types, is it possible to determine derivative types?
        out_dim = len(self.output_variables)
        in_dim = len(self.input_variables)
        results = []

        for i in range(out_dim):
            if not self.output_variables[i].is_differentiable():
                continue
            for j in range(in_dim):
                if not self.input_variables[j].is_differentiable():
                    continue
                for k in range(j, in_dim):
                    if not self.input_variables[k].is_differentiable():
                        continue
                    node_derivative_name = \
                        _node_derivative_prefix + self.second_order_derivative_name(
                            self.output_variables[i].nick_name,
                            self.input_variables[j].nick_name,
                            self.input_variables[k].nick_name)
                    node_derivative_type = self.output_variables[i].var_type
                    results.append(((i, j, k), node_derivative_name, node_derivative_type))

        return results

    # TODO() re implement get_first_order_derivatives_info to avoid returning too much
    def get_first_order_derivative_name_and_var_type(self, output_idx, input_idx):
        return _node_derivative_prefix + self.first_order_derivative_name(
            self.output_variables[output_idx].nick_name, self.input_variables[input_idx].nick_name), \
               self.output_variables[output_idx].var_type

    def get_result_names_and_types(self, enable_1st_order_derivative: bool = False,
                                   enable_2nd_order_derivative: bool = False):
        result_names = []
        result_types = []
        out_dim = len(self.output_variables)
        for i in range(out_dim):
            result_names.append(self.output_variables[i].nick_name)
            result_types.append(self.output_variables[i].var_type)

        # 1st order:
        if enable_1st_order_derivative:
            first_order_infos = self.get_first_order_derivatives_info()
            for _, name, var_type in first_order_infos:
                result_names.append(name)
                result_types.append(var_type)

        # 2nd order:
        if enable_2nd_order_derivative:
            second_order_infos = self.get_second_order_derivatives_info()
            for _, name, var_type in second_order_infos:
                result_names.append(name)
                result_types.append(var_type)
        return result_names, result_types


class FunctionBase:
    """
    A function is a certain type of process on Variables.
    For detailed implementation of the function, it really depends on what kind of Variables
    is it processing. From such consideration the code generation may vary.
    It is not only the var_type of the Variables that matters, but also what they are:
    Are they inputs, or local variables? Anyway, code generation takes that very specific
    info.
    A function can be an abstract notion, a symbol that matches multiple implementation,
    if without context.

    This is somehow like what a compiler is doing.
    """

    def __init__(self, input_spec: List[str], output_spec: List[str]):
        """
        Use '' to make function infer the type during call.
        :param output_spec: ['double','double']
        :param input_spec: ['double','float','']
        """

        assert all([(var_type in Variable.var_types or var_type == '') for var_type in input_spec]), \
            "invalid input spec"
        assert all([(var_type in Variable.var_types or var_type == '') for var_type in output_spec]), \
            "invalid output spec"

        self.output_spec = output_spec.copy()
        self.input_spec = input_spec.copy()

    def __call__(self, *args, **kwargs):
        # Check graph consistency
        graph: "Graph" = None
        for element in args:
            if type(element) is Variable:
                if graph is None:
                    graph = element.graph
                else:
                    assert graph is element.graph, "Function can't be called on variables from different graphs."
        assert graph is not None, "Can't infer graph from function call."

        # Create input variables
        input_variables: List[Variable] = []
        for element in args:
            if type(element) is Variable:
                input_variables.append(element)
            elif type(element) in [float, int]:
                unnamed_variable = graph.create_un_named_variable()
                unnamed_variable.defined_as_constant(element, Variable.default_var_type)
                input_variables.append(unnamed_variable)

        # Infer output var_type
        input_combined_var_type = Variable.infer_combined_type(
            [input_variable.var_type for input_variable in input_variables])

        # Create output variables
        output_variables: List[Variable] = []
        for i in range(len(self.output_spec)):
            var_type = input_combined_var_type if self.output_spec[i] == '' else self.output_spec[i]
            output_variable = graph.create_un_named_variable()
            output_variable.defined_as_expr(self, input_variables, i, var_type)
            output_variables.append(output_variable)

        # check the context compatibility
        constext = FunctionContext(input_variables, output_variables)
        res, dbg = self.is_compatible(constext)
        assert res, dbg

        # Inform graph the operation.
        graph.append_operation(self, constext)

        if len(output_variables) > 1:
            return tuple(output_variables)
        else:
            return output_variables[0]

    def is_compatible(self, context: FunctionContext) -> Tuple[bool, str]:
        """
        :param context:
        :return: result, debug_string
        """
        if len(self.output_spec) != len(context.output_variables):
            return False, "output variable number miss-match"
        if len(self.input_spec) != len(context.input_variables):
            return False, "input variable number miss-match"
        # At here, we don't check the input/output type match the interface.
        # That whether a certain variable can be transferred to this function, is decided by
        # the derived class.
        for output_var in context.output_variables:
            if output_var.type not in [Variable.TYPE_STATE_EXPR, Variable.TYPE_CONFIG_EXPR]:
                return False, "output variables must be expr"
        return True, ""

    def print_definition(self, context: FunctionContext) -> List[str]:
        """
        What appears before the graph.
        For e.g. for sympy nodes with large result, we want to wrap them.

        Will enable this when:
        :param inputs: a, b, c
        :param outputs: sum, mult
        :return: List[str] as lines.

        inline ArrayNd GetSumMultFromABC(double a, const ArrayNd& b, double c) {
            ...
            ...
        }

        """
        return []

    def print_call(self, context: FunctionContext, enable_1st_order_derivative: bool = False,
                   enable_2nd_order_derivative: bool = False) -> List[str]:
        """
        Will:
        1, assume the inputs are ready.
        2, get outputs, together with their node derivatives.
        :param inputs: a, b, c
        :param outputs: sum
        :return: List[str] as lines.
        double sum = a + b + c
        (When enabling derivative)
        double D_sum_D_a = 1;
        double D_sum_D_b = 1;

        or:
        double sum, D_sum_D_a,D_sum_D_b,...
        (When enabling derivative)
        sum = SumWithDerivative(a, b, &D_sum_D_a, &D_sum_D_b)

        :param context:
        :param enable_1st_order_derivative:
        :param enable_2nd_order_derivative:
        :return: result types, result names, and implementation lines
        """
        assert False, "not implemented"

    def demanded_const_references(self, context: FunctionContext) -> List[Tuple[str, str]]:
        """
        What appears as the external dependency of the graph:
        passed to each graph function.

        can't share same name with variables.
        :param inputs:
        :param outputs:
        :return: [(CostSpace, space1), (CostSpace, space2)...]
        """
        return []


# A convention:
# When Function deems whether an input is derivative-interested by itself.
# For a derivative that is not printed, it would be 0.0.
# A function also need to make sure what it prints is not pre-defined.
# this is achieved by assuming the naming unique-ness of input variables.

# What print call do: provide the

class SymPyFunction(FunctionBase):
    def __init__(self, sympy_function, output_dim_override=None, input_dim_override=None):
        # infer the number of sympy function inputs.
        if input_dim_override is not None:
            input_dim = input_dim_override
        else:
            input_dim = sympy_function.__code__.co_argcount
        assert input_dim > 0, "Sympy function should have at least one input"

        output_dim = 1
        if output_dim_override is not None:
            assert type(output_dim_override) is int
            output_dim = output_dim_override
        else:
            # We infer the number of the sympy function outputs.
            # using the following hacky method:
            try:
                a_test_input = [1.0] * input_dim
                test_output = sympy_function(*a_test_input)
                if type(test_output) is tuple or type(test_output) is list:
                    output_dim = len(test_output)
            except:
                assert False, "Failed to infer sympy function output dimension, please specify 'output_dim_override'."

        assert output_dim > 0, "Sympy function should have at least one output"

        input_spec = [''] * input_dim
        output_spec = [''] * output_dim

        super(SymPyFunction, self).__init__(input_spec, output_spec)
        self.sympy_function = sympy_function

    def print_call(self, context: FunctionContext, enable_1st_order_derivative: bool = False,
                   enable_2nd_order_derivative: bool = False) -> List[str]:

        res, dbg = self.is_compatible(context)
        assert res, dbg

        input_symbols = []
        for input_variable in context.input_variables:
            if input_variable.type is Variable.TYPE_CONSTANT:
                input_symbols.append(input_variable.value)
            else:
                input_symbols.append(sp.symbols(input_variable.nick_name))

        out_dim = len(context.output_variables)
        in_dim = len(context.input_variables)

        # distinguish MIMO, derivatives
        result_exprs = []
        result_names = []
        result_types = []

        # Zero order:
        output_exprs = self.sympy_function(*input_symbols)
        if len(context.output_variables) == 1:
            assert type(output_exprs) is not tuple
            output_exprs = [output_exprs, ]
        else:
            assert type(output_exprs) is tuple or type(output_exprs) is list
        assert len(output_exprs) == out_dim

        result_exprs += list(output_exprs)
        for i in range(out_dim):
            result_names.append(context.output_variables[i].nick_name)
            result_types.append(context.output_variables[i].var_type)

        # 1st order:
        if enable_1st_order_derivative:
            first_order_infos = context.get_first_order_derivatives_info()
            for out_idx_in_idx, name, var_type in first_order_infos:
                out_idx, in_idx = out_idx_in_idx
                result_names.append(name)
                result_exprs.append(output_exprs[out_idx].diff(input_symbols[in_idx]))
                result_types.append(var_type)

        # 2nd order:
        if enable_2nd_order_derivative:
            second_order_infos = context.get_second_order_derivatives_info()
            for out_idx_in_idx1_in_idx2, name, var_type in second_order_infos:
                out_idx, in_idx1, in_idx2 = out_idx_in_idx1_in_idx2
                result_names.append(name)
                result_exprs.append((output_exprs[out_idx].diff(input_symbols[in_idx1])).diff(input_symbols[in_idx2]))
                result_types.append(var_type)

        replacements, reduced_exprs = sp.cse(result_exprs, sp.utilities.iterables.numbered_symbols(_sympy_var_prefix))

        assert type(reduced_exprs) is tuple or type(reduced_exprs) is list
        assert len(result_exprs) == len(reduced_exprs)

        printer = OptimizedCXX11Printer()

        sympy_local_var_type = \
            Variable.infer_combined_type([input_variable.var_type for input_variable in context.input_variables])

        lines = []
        # calculation steps
        lines.append("{")
        for replacement in replacements:
            line = printer.doprint(replacement[1], sympy_local_var_type + ' ' + str(replacement[0])).replace('\n', '')
            lines.append(_indent + line)
        # assign outputs
        for i in range(len(reduced_exprs)):
            line = printer.doprint(reduced_exprs[i], result_names[i])
            lines.append(_indent + line)
        lines.append("}")

        return lines


class _SymPyOperatorFunctions:
    sympy_add = SymPyFunction(lambda a, b: a + b)
    sympy_sub = SymPyFunction(lambda a, b: a - b)
    sympy_mul = SymPyFunction(lambda a, b: a * b)
    sympy_pow = SymPyFunction(lambda a, b: a ** b)
    sympy_div = SymPyFunction(lambda a, b: a / b)


class CppNamespaceFunction(FunctionBase):
    def __init__(self, function_name, input_spec: List[str], output_spec: List[str]):
        # input , output types must be specified
        assert all([var_type in Variable.var_types for var_type in input_spec]), "invalid input spec"
        assert all([var_type in Variable.var_types for var_type in output_spec]), "invalid output spec"

        super(CppNamespaceFunction, self).__init__(input_spec, output_spec)
        assert _is_valid_namespace(function_name), "invalid function_name :(%s)" % function_name
        self.function_name = function_name


class CppMemberFunction(FunctionBase):
    def __init__(self, object_type: str, object_name: str, function_name: str, input_spec: List[str],
                 output_spec: List[str]):
        """
        const CostSpace& space1;
        space1.collision_cost(...)
        :param object_type: CostSpace
        :param object_name: space1
        :param function_name: collision_cost
        """
        # input , output types must be specified
        assert all([var_type in Variable.var_types for var_type in input_spec]), "invalid input spec"
        assert all([var_type in Variable.var_types for var_type in output_spec]), "invalid output spec"

        super(CppMemberFunction, self).__init__(input_spec, output_spec)

        assert _is_valid_namespace(object_type), "invalid object_type :(%s)" % object_type
        assert _is_valid_cpp_name(object_name), "invalid object_name :(%s)" % object_name
        assert _is_valid_cpp_name(function_name), "invalid function_name :(%s)" % function_name

        self.object_type = object_type
        self.object_name = object_name
        self.function_name = function_name


class GraphFunction(FunctionBase):
    # So people can be owner of a sub graph, managed by a big graph.
    # TODO(): implement with :
    #  1, checking no - loop dependency
    #  2, assert each outer graph statefuls are not passed as config to this graph.
    pass


class ConditionalFunction(FunctionBase):
    # So we can avoid running a piece of code, by using gflag int.
    # We only support gflag int.
    # TODO(): implement with :
    #  1, checking no - loop dependency
    #  2, it is a little tricky, since each sub function are defining the output vars by themselves,
    #  There can be disagreement in the output var_types and even existence.
    #  That we need to merge them, their result types and names.
    pass


class Graph:
    def __init__(self, name: str = "UntitledFunc"):
        # contain all variables
        # from nick_name to Variable
        self.name = name
        self._all_variables: Dict[str, Variable] = {}

        self._un_named_count = 0

        self._operations: List[Tuple[FunctionBase, FunctionContext]] = []

    def state_inputs(self, names: List[str], var_type: str):
        assert var_type in Variable.var_types, "Invalid type: %s" % var_type
        if not names:
            return None

        results = [self.create_new_variable(name) for name in names]
        for var in results:
            var.defined_as_state_input(var_type)

        if len(results) == 1:
            return results[0]
        else:
            return results

    def config_inputs(self, names: List[str], var_type: str):
        assert var_type in Variable.var_types, "Invalid type: %s" % var_type
        if not names:
            return None

        results = [self.create_new_variable(name) for name in names]
        for var in results:
            var.defined_as_config_input(var_type)

        if len(results) == 1:
            return results[0]
        else:
            return results

    def append_operation(self, function: FunctionBase, context: FunctionContext):
        assert function.is_compatible(context)
        # TODO(): check function references are good with variable names.
        self._operations.append((function, context))

    #

    def get_state_input_variables(self):
        res = []
        for variable in self._all_variables.values():
            if variable.type is Variable.TYPE_STATE_INPUT:
                res.append(variable)
        return res

    def get_config_input_variables(self):
        res = []
        for variable in self._all_variables.values():
            if variable.type is Variable.TYPE_CONFIG_INPUT:
                res.append(variable)
        return res

    def is_member(self, variable: Variable):
        if not variable.graph is self:
            return False
        if not variable.nick_name in self._all_variables:
            return False
        if not self._all_variables[variable.nick_name] is variable:
            return False
        return True

    @staticmethod
    def get_first_order_graph_derivatives_info(input_variables: List[Variable], output_variables: List[Variable]):
        """
        :return: a list of tuple:
         ((output_idx: int, input_idx: int), graph_derivative_name: str, graph_derivative_type: str)
        """
        out_dim = len(output_variables)
        in_dim = len(input_variables)
        results = []

        for i in range(out_dim):
            if not output_variables[i].is_differentiable():
                continue
            for j in range(in_dim):
                if not input_variables[j].is_differentiable():
                    continue
                graph_derivative_name = \
                    _graph_derivative_prefix + "D_%s_D_%s" % (
                        output_variables[i].nick_name, input_variables[j].nick_name)
                graph_derivative_type = output_variables[i].var_type

                results.append(((i, j), graph_derivative_name, graph_derivative_type))

        return results

    @staticmethod
    def get_second_order_graph_derivatives_info(input_variables: List[Variable], output_variables: List[Variable]):
        """
        :return: a list of tuple:
         ((output_idx: int, input_idx: int, input_idx: int), graph_derivative_name: str, graph_derivative_type: str)
        """

        out_dim = len(output_variables)
        in_dim = len(input_variables)
        results = []

        for i in range(out_dim):
            if not output_variables[i].is_differentiable():
                continue
            for j in range(in_dim):
                if not input_variables[j].is_differentiable():
                    continue
                for k in range(j, in_dim):
                    if not input_variables[k].is_differentiable():
                        continue
                    graph_derivative_name = \
                        _graph_derivative_prefix + "D2_%s_D_%s_D_%s" % (
                            output_variables[i].nick_name,
                            input_variables[j].nick_name,
                            input_variables[k].nick_name)
                    graph_derivative_type = output_variables[i].var_type
                    results.append(((i, j, k), graph_derivative_name, graph_derivative_type))

        return results

    def print_zero_order_definition(self, output_variables: List[Variable]):
        """
        deprecated
        :param output_variables:
        :return:
        """
        for variable in output_variables:
            assert variable.graph is self
            assert variable in self._all_variables.values()

        lines = []

        state_inputs = []
        config_inputs = []
        outputs = []
        for variable in self._all_variables.values():
            if variable.type is Variable.TYPE_CONFIG_INPUT:
                config_inputs.append((variable.var_type, variable.nick_name))
            elif variable.type is Variable.TYPE_STATE_INPUT:
                state_inputs.append((variable.var_type, variable.nick_name))

            if variable in output_variables:
                outputs.append((variable.var_type, variable.nick_name))

        function_head = 'void ' + _decorate_function_name_with_derivatives(self.name, False, False) + "("
        for var_type, name in config_inputs:
            function_head += Variable.const_reference(var_type) + ' ' + name + ','
        for var_type, name in state_inputs:
            function_head += Variable.const_reference(var_type) + ' ' + name + ','
        for var_type, name in outputs:
            function_head += var_type + '* ' + _graph_output_prefix + name + ','
        function_head = function_head[:-1]
        function_head += ") {"

        lines.append(function_head)

        for function, context in self._operations:
            result_types, result_names, calculation = function.print_call(context, False, False)
            # TODO(): clear unused variables making use of AC automaton.
            # Or: wrap sympy function

            for i in range(len(result_types)):
                lines.append(_indent + result_types[i] + ' ' + result_names[i] + ';')
            lines += [_indent + ln for ln in calculation]

        for var_type, name in outputs:
            lines.append(_indent + '*' + _graph_output_prefix + name + '=' + name + ';')
        lines.append("}")
        return lines

    def print_derivative_definition(self, output_variables: List[Variable],
                                    enable_1st_order_derivative: bool = False,
                                    enable_2nd_order_derivative: bool = False):
        # BP core:
        # 1, set of declared vars. check existence when using. special simplification when absence
        # 2, when creating new declared vars, check existence for addition.
        for variable in output_variables:
            assert self.is_member(variable), "output variables must be member of the graph."

        state_inputs = self.get_state_input_variables()
        config_inputs = self.get_config_input_variables()
        # output_variables
        input_variables = state_inputs + config_inputs

        first_order_info = []
        second_order_info = []
        if enable_1st_order_derivative:
            first_order_info = self.get_first_order_graph_derivatives_info(input_variables, output_variables)

        if enable_2nd_order_derivative:
            second_order_info = self.get_second_order_graph_derivatives_info(input_variables, output_variables)

        # Determine function head
        function_head = 'void ' + _decorate_function_name_with_derivatives(self.name, enable_1st_order_derivative,
                                                                           enable_2nd_order_derivative) + "("
        for var in state_inputs:
            function_head += Variable.const_reference(var.var_type) + ' ' + var.nick_name + ' /*state*/,'
        for var in config_inputs:
            function_head += Variable.const_reference(var.var_type) + ' ' + var.nick_name + ' /*config*/,'
        for var in output_variables:
            function_head += var.var_type + '* ' + _graph_output_prefix + var.nick_name + ','

        if enable_1st_order_derivative:
            for _, name, var_type in first_order_info:
                function_head += var_type + '* ' + _graph_output_prefix + name + ','

        if enable_2nd_order_derivative:
            for _, name, var_type in second_order_info:
                function_head += var_type + '* ' + _graph_output_prefix + name + ','

        if function_head[-1] == ',':
            function_head = function_head[:-1]

        function_head += ") {"

        lines = []
        lines.append(function_head)

        # when not in this set, the field is deemed to be 0
        # an exception is, for da_da, that value is 1.
        existing_fields = set()

        for function, context in self._operations:
            calculation = \
                function.print_call(context,
                                    enable_2nd_order_derivative or enable_1st_order_derivative,
                                    enable_2nd_order_derivative)
            # TODO(): clear unused variables making use of AC automaton.
            result_names, result_types = context.get_result_names_and_types(
                enable_2nd_order_derivative or enable_1st_order_derivative,
                enable_2nd_order_derivative)

            for i in range(len(result_types)):
                lines.append(_indent + result_types[i] + ' ' + result_names[i] + ';')
            lines += [_indent + ln for ln in calculation]

            # Assume the context provides follows:
            if enable_2nd_order_derivative or enable_1st_order_derivative:
                for _, node_der_name, _ in context.get_first_order_derivatives_info():
                    existing_fields.add(node_der_name)
            if enable_2nd_order_derivative:
                for _, node_der_name, _ in context.get_second_order_derivatives_info():
                    existing_fields.add(node_der_name)

        for var in output_variables:
            lines.append(_indent + '*' + _graph_output_prefix + var.nick_name + '=' + var.nick_name + ';')

        def add_to_field(field_name: str, field_var_type: str, expression_str):
            if expression_str == '':
                return

            if field_name in existing_fields:
                lines.append(_indent + field_name + '+=' + expression_str + ';')
            else:
                lines.append(_indent + field_var_type + ' ' + field_name + '=' + expression_str + ';')
                existing_fields.add(field_name)

        class Expression:
            def expression(self) -> str:
                pass

            def is_constant_one(self):
                pass

            def is_constant_zero(self):
                pass

        class ExistingExpression(Expression):
            # TODO(): also simplify for special node_derivatives outputs such as 1.0, 0.0;
            def __init__(self, expr: str):
                self.expr = expr

            def is_constant_one(self):
                return False

            def is_constant_zero(self):
                return self.expr not in existing_fields

            def expression(self):
                return self.expr

        class GraphDerivative(Expression):
            def __init__(self, out_name: str, in_names: List[str]):
                self.out_name = out_name
                self.in_names = in_names
                self.in_names.sort()
                self.order = len(self.in_names)
                assert self.order in [1, 2]

                if self.order is 1:
                    self.expr: str = _graph_derivative_prefix + FunctionContext.first_order_derivative_name(
                        self.out_name, self.in_names[0])
                elif self.order is 2:
                    self.expr: str = _graph_derivative_prefix + FunctionContext.second_order_derivative_name(
                        self.out_name, *self.in_names)

            def expression(self):
                return self.expr

            def is_constant_one(self):
                if self.expr not in existing_fields:
                    if self.order is 1:
                        return self.in_names[0] == self.out_name
                return False

            def is_constant_zero(self):
                if self.expr not in existing_fields and not self.is_constant_one():
                    return True
                return False

        class NumericExpression(Expression):
            def __init__(self, val: float):
                self.val = val

            def is_constant_one(self):
                return self.val == 1.0

            def is_constant_zero(self):
                return self.val == 0.0

            def expression(self) -> str:
                return str(self.val)

        def get_product_expression(items: List[Expression]):
            if not items:
                return ''

            if any([item.is_constant_zero() for item in items]):
                return ''

            if all([item.is_constant_one() for item in items]):
                return '1'

            # no zero, have non one item
            result = ''
            for item in items:
                if not item.is_constant_one():
                    result += '*' + item.expression()
            result = result[1:]
            return result

        if enable_1st_order_derivative or enable_2nd_order_derivative:
            # Compute first order derivatives
            for active_variable in output_variables:
                if not active_variable.is_differentiable():
                    continue
                # determine derivative types
                active_var_type = active_variable.var_type

                # skip steps after getting the active variable
                step = len(self._operations) - 1
                while step >= 0 and active_variable not in self._operations[step][1].output_variables:
                    step -= 1

                # bp the entire graph
                for i in range(step, -1, -1):
                    _, context = self._operations[i]

                    # Update graph derivative from Node derivatives
                    for out_idx_in_idx, node_der_name, _ in context.get_first_order_derivatives_info():
                        out_idx, in_idx = out_idx_in_idx
                        in_name = context.input_variables[in_idx].nick_name
                        out_name = context.output_variables[out_idx].nick_name

                        # Simple chain rule
                        d_active_d_node_in = GraphDerivative(active_variable.nick_name, [in_name, ])
                        d_active_d_node_out = GraphDerivative(active_variable.nick_name, [out_name, ])
                        node_d_out_d_in = ExistingExpression(node_der_name)

                        add_to_field(d_active_d_node_in.expression(), active_var_type,
                                     get_product_expression([d_active_d_node_out, node_d_out_d_in]))

        if enable_2nd_order_derivative:
            # A table of co-relation.
            # ensures a not in table[a]
            def co_relate(name_1, name_2, table: Dict[str, Set[str]]):
                if name_1 == name_2:
                    return
                if name_1 not in table:
                    table[name_1] = set()
                if name_2 not in table:
                    table[name_2] = set()

                table[name_1].add(name_2)
                table[name_2].add(name_1)

            for active_variable in output_variables:
                if not active_variable.is_differentiable():
                    continue

                # The cross items of a certain variable
                existing_cross_items_of_variable: Dict[str, Set[str]] = {}

                # determine derivative types
                active_var_type = active_variable.var_type

                # skip steps after getting the active variable
                step = len(self._operations) - 1
                while step >= 0 and active_variable not in self._operations[step][1].output_variables:
                    step -= 1

                # bp the entire graph
                for i in range(step, -1, -1):
                    _, context = self._operations[i]

                    # Update graph derivative from Node derivatives
                    for out_idx_in_idx_1_in_idx_2, node_der_name, _ in context.get_second_order_derivatives_info():
                        out_idx, in_idx_1, in_idx_2 = out_idx_in_idx_1_in_idx_2
                        in_1_name = context.input_variables[in_idx_1].nick_name
                        in_2_name = context.input_variables[in_idx_2].nick_name
                        out_name = context.output_variables[out_idx].nick_name

                        d2_active_d_node_in_1_d_node_in_2 = \
                            GraphDerivative(active_variable.nick_name, [in_1_name, in_2_name])
                        d_active_d_node_out = GraphDerivative(active_variable.nick_name, [out_name, ])
                        d2_active_d_node_out_d_node_out = \
                            GraphDerivative(active_variable.nick_name, [out_name, out_name])

                        node_d2_out_d_in_1_d_in_2 = ExistingExpression(node_der_name)
                        node_d_out_d_in_1 = ExistingExpression(
                            context.get_first_order_derivative_name_and_var_type(out_idx, in_idx_1)[0])
                        node_d_out_d_in_2 = ExistingExpression(
                            context.get_first_order_derivative_name_and_var_type(out_idx, in_idx_2)[0])

                        add_to_field(d2_active_d_node_in_1_d_node_in_2.expression(), active_var_type,
                                     get_product_expression([d_active_d_node_out, node_d2_out_d_in_1_d_in_2]))
                        add_to_field(d2_active_d_node_in_1_d_node_in_2.expression(), active_var_type,
                                     get_product_expression(
                                         [d2_active_d_node_out_d_node_out, node_d_out_d_in_1, node_d_out_d_in_2]))
                        if in_idx_1 != in_idx_2 and \
                                d2_active_d_node_in_1_d_node_in_2.expression() in existing_fields:
                            co_relate(in_1_name, in_2_name, existing_cross_items_of_variable)

                    for out_idx_in_idx, node_der_name, _ in context.get_first_order_derivatives_info():
                        out_idx, in_idx = out_idx_in_idx
                        in_name = context.input_variables[in_idx].nick_name
                        out_name = context.output_variables[out_idx].nick_name

                        # output channel has no co-related items
                        if out_name not in existing_cross_items_of_variable:
                            continue

                        node_d_out_d_in = ExistingExpression(node_der_name)
                        for co_related in existing_cross_items_of_variable[out_name]:
                            d2_active_d_node_out_d_co_related = \
                                GraphDerivative(active_variable.nick_name, [out_name, co_related])
                            d2_active_d_node_in_d_co_related = \
                                GraphDerivative(active_variable.nick_name, [in_name, co_related])
                            if co_related != in_name:
                                add_to_field(d2_active_d_node_in_d_co_related.expression(), active_var_type,
                                             get_product_expression(
                                                 [d2_active_d_node_out_d_co_related, node_d_out_d_in]))
                            else:
                                add_to_field(d2_active_d_node_in_d_co_related.expression(), active_var_type,
                                             get_product_expression(
                                                 [NumericExpression(2), d2_active_d_node_out_d_co_related,
                                                  node_d_out_d_in]))

                            if co_related != in_name and \
                                    d2_active_d_node_in_d_co_related.expression() in existing_fields:
                                co_relate(in_name, co_related, existing_cross_items_of_variable)

        # output fields
        if enable_1st_order_derivative:
            for out_idx_in_idx, name, var_type in first_order_info:
                out_idx, in_idx = out_idx_in_idx
                graph_d_out_d_in = \
                    GraphDerivative(output_variables[out_idx].nick_name, [input_variables[in_idx].nick_name, ])
                expression = get_product_expression([graph_d_out_d_in, ])
                if expression == '':
                    expression = '0'
                lines.append(_indent + '*' + _graph_output_prefix + name + '=' +
                             expression + ';')

        if enable_2nd_order_derivative:
            for out_idx_in_idx_1_in_idx_2, name, var_type in second_order_info:
                out_idx, in_idx_1, in_idx_2 = out_idx_in_idx_1_in_idx_2
                graph_d2_out_d_in_1_d_in_2 = \
                    GraphDerivative(output_variables[out_idx].nick_name,
                                    [input_variables[in_idx_1].nick_name,
                                     input_variables[in_idx_2].nick_name])
                expression = get_product_expression([graph_d2_out_d_in_1_d_in_2, ])
                if expression == '':
                    expression = '0'
                lines.append(_indent + '*' + _graph_output_prefix + name + '=' +
                             expression + ';')

        lines.append("}")
        return lines

    def create_new_variable(self, name: str):
        new_var = self.create_un_named_variable()
        res, dbg = self.re_name(new_var.nick_name, name)
        assert res, dbg

        return new_var

    def re_name(self, old_nick_name: str, new_name: str):
        if old_nick_name not in self._all_variables.keys():
            return False, "\"%s\" not found" % old_nick_name
        new_nick_name = self._get_nick_name(new_name)
        if new_nick_name is None:
            return False, "Invalid name: %s" % new_name
        if new_nick_name in self._all_variables.keys():
            return False, "Nick name \"%s\"->\"%s\" already exist for \"%s\"" % (
                new_name, new_nick_name, self._all_variables[new_nick_name].name)
        for sub_string in _built_in_name_sub_strings:
            if new_nick_name.find(sub_string) >= 0:
                return False, "Nick name \"%s\"->\"%s\" contains built-in sub string: \"%s\"" % (
                    new_name, new_nick_name, sub_string)

        var = self._all_variables[old_nick_name]

        var.name = new_name
        var.nick_name = new_nick_name
        self._all_variables[new_nick_name] = var
        self._all_variables.pop(old_nick_name)
        return True, ""

    def create_un_named_variable(self):
        nick_name = _unnamed_graph_var_prefix + "%d" % self._un_named_count
        self._un_named_count += 1

        for sub_string in _built_in_name_sub_strings:
            assert nick_name.find(
                sub_string) < 0 or sub_string == _unnamed_graph_var_prefix, "Nick name \"%s\" contains built-in sub string: \"%s\"" % (
                nick_name, sub_string)

        new_variable = Variable(nick_name, nick_name, self)
        self._all_variables[nick_name] = new_variable

        return new_variable

    @staticmethod
    def _get_nick_name(name: str):
        """
        generate a string that is suitable for variable naming as a coding variable
        :param name:
        :return:
        """
        name = name.replace(' ', '_')
        name = name.lower()

        result = ''
        count = 0
        for c in name:
            if c == '_' or c in _lower_alphabet_chars or (count > 0 and c in _number_chars):
                result += c
            else:
                result += 'x'
            count += 1

        return result if len(result) > 0 else None


def complex_sympy_function(a, b):
    f = a ** 2 + b ** 2 + 2 * a * b
    return f


def radius_func(a, b):
    tp = (a ** 2 + b ** 2) ** 0.5 + a * b
    return tp * (a) ** 0.5


g = Graph(name='Radius')

x, y = g.state_inputs(['x', 'y'], 'double')
radius = SymPyFunction(radius_func)

z = radius(x, y)

"""
a = x
b = y
tp = (a**2 +b**2)**0.5 +  a * b
z = tp * a**0.5
"""

z.set_name('z')
lines = g.print_derivative_definition([z], True, True)
s = ''
for line in lines:
    s += line
    s += '\n'
print(s)

# TODO(): test validity in C++ project. operators. Bp process for gradients. (single output)
