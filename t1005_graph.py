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
_graph_constant_prefix = "G_CONSTANT_"
_graph_unused_prefix = "G_UNUSED_"
_built_in_name_sub_strings = [_sympy_var_prefix, _unnamed_graph_var_prefix, _graph_output_prefix]

_indent = "  "


def _full_output_channels_with_derivatives(
        in_dim: int,
        out_dim: int,
        enable_1st_order_derivative: bool = False,
        enable_2nd_order_derivative: bool = False) -> List[Tuple]:
    """
    :param in_dim: input dimension
    :param out_dim: output dimension
    :param enable_1st_order_derivative:
    :param enable_2nd_order_derivative:
    :return: tuple of int representing the order and the channel of the output:
    (i,) Out[i].
    (i,j) d_Out[i]_d_In[j]
    (i,j,k) d2_Out[i]_d_In[j]_d_In[k]
    """
    #
    channels = []

    for i in range(out_dim):
        channels.append((i,))

    if enable_1st_order_derivative:
        for i in range(out_dim):
            for j in range(in_dim):
                channels.append((i, j))

    if enable_2nd_order_derivative:
        for i in range(out_dim):
            for j in range(in_dim):
                for k in range(j, in_dim):
                    channels.append((i, j, k))

    return channels


def _get_channel_name(output_and_input_names: Tuple):
    """

    :param output_and_input_names: a tuple with length 1,2, or 3
    :return:
    (out,) out
    (out,in1) D_out_D_in1
    (out,in1,in2) D2_out_D_in1_D_in2
    """
    # in case some uses list
    names = tuple(output_and_input_names)

    l = len(names)

    assert all([_is_valid_lower_case_cpp_name(name) for name in names]), "_get_channel_name: invalid input name."

    if l == 1:
        return names[0]
    elif l == 2:
        return "D_%s_D_%s" % names
    elif l == 3:
        return "D2_%s_D_%s_D_%s" % names
    else:
        assert False, "_get_channel_name: invalid input length."


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


def _is_valid_lower_case_cpp_name(cpp_name: str):
    count = 0
    for c in cpp_name:
        if (count > 0 and c in _number_chars) or \
                c in _lower_alphabet_chars or \
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
    # numerical_var_types = ['ArrayXd', 'double', 'float', 'int']
    # numerical_var_types_set = set(numerical_var_types)

    # A more complex type can be initialized from a less complex one.
    # types with equal complexity are inter-tranferable.
    numerical_var_type_complexity = {'int': 1,
                                     'float': 2,
                                     'double': 3,
                                     'ArrayXd': 4}

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
        assert self.is_numerical_var_type(var_type), "Invalid type: %s" % var_type

        self.type = self.TYPE_STATE_INPUT
        self.var_type = var_type

    def defined_as_constant(self, value: float, var_type: str = 'double'):
        """
        :param value:
        :param var_type:
        :return:
        """
        assert self.type is self.TYPE_UN_DEFINED, "re definition not allowed"
        assert self.is_numerical_var_type(var_type), "Invalid type: %s" % var_type

        self.value = value
        self.type = self.TYPE_CONSTANT
        self.var_type = var_type

    def defined_as_config_input(self, var_type: str = 'double'):
        """
        :param var_type:
        :return:
        """
        assert self.type is self.TYPE_UN_DEFINED, "re definition not allowed"
        assert _is_valid_cpp_name(var_type), "Invalid type: %s" % var_type

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
        assert self.is_numerical_var_type(var_type), "Invalid type: %s" % var_type

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
    def infer_combined_var_type(cls, var_type_list: List[str]):

        combined = None

        for var_type in var_type_list:
            if var_type not in cls.numerical_var_type_complexity:
                continue
            if combined is None:
                combined = var_type
            else:
                if cls.numerical_var_type_complexity[var_type] > cls.numerical_var_type_complexity[combined]:
                    combined = var_type

        return combined

    @classmethod
    def is_transferable(cls, from_type, to_type):
        if from_type not in cls.numerical_var_type_complexity:
            return False

        if to_type not in cls.numerical_var_type_complexity:
            return False

        return cls.numerical_var_type_complexity[from_type] <= cls.numerical_var_type_complexity[to_type]

    @classmethod
    def is_numerical_var_type(cls, var_type):
        return var_type in cls.numerical_var_type_complexity

    @classmethod
    def const_reference(cls, var_type: str):
        assert _is_valid_cpp_name(var_type)
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


class Context:
    """
    Function context specifies the major input output action.

    For how to call the function, it depends on the class.
    """

    def __init__(self, inputs: List[Variable], outputs: List[Variable]):
        self.input_variables = inputs
        self.output_variables = outputs


class Option:
    def __init__(self,
                 enable_1st_order_derivative: bool = False,
                 enable_2nd_order_derivative: bool = False):
        self.enable_1st_order_derivative = enable_1st_order_derivative
        self.enable_2nd_order_derivative = enable_2nd_order_derivative

    def __eq__(self, other: "Option"):
        return all([
            self.enable_1st_order_derivative == other.enable_1st_order_derivative,
            self.enable_2nd_order_derivative == other.enable_2nd_order_derivative
        ])

    def decorate(self, function_name: str):
        assert _is_valid_cpp_name(function_name)
        mask = [self.enable_1st_order_derivative, self.enable_2nd_order_derivative]
        titles = ['First', 'Second']

        result = function_name
        if any(mask):
            result += 'With'
            for i in range(len(mask)):
                if mask[i]:
                    result += titles[i]
            result += 'OrderDerivatives'

        # TODO: other options.

        return result


# options to be considered during code generation.
_all_options = [
    Option(False, False),
    Option(True, False),
    Option(True, True),
]


class FullContext:
    def __init__(self, context: Context,
                 option: Option):
        self.context = context
        self.option = option
        # and other options maybe

        self.zero_order_channels: List[Tuple[int]] = []
        self.first_order_channels: List[Tuple[int, int]] = []
        self.second_order_channels: List[Tuple[int, int, int]] = []
        self._init_output_channels()

    def _init_output_channels(self):
        """
        keep only the differentiable output channels
         0 order non-differentiable also kept.
        :return: List[Tuple]
        """
        out_dim = len(self.context.output_variables)
        in_dim = len(self.context.input_variables)

        full_output_channels = _full_output_channels_with_derivatives(in_dim, out_dim,
                                                                      self.option.enable_1st_order_derivative,
                                                                      self.option.enable_2nd_order_derivative)

        for channel in full_output_channels:
            # keep only the differentiable output channels
            # 0 order non-differentiable also kept.
            should_avoid_channel = False
            if len(channel) > 1:
                if not self.context.output_variables[channel[0]].is_differentiable():
                    should_avoid_channel = True

                for i in range(1, len(channel)):
                    if not self.context.input_variables[channel[i]].is_differentiable():
                        should_avoid_channel = True
            else:
                # 0 order channel always kept.
                pass
            if should_avoid_channel:
                continue

            if len(channel) == 1:
                self.zero_order_channels.append(channel)
            elif len(channel) == 2:
                self.first_order_channels.append(channel)
            elif len(channel) == 3:
                self.second_order_channels.append(channel)

    def get_output_channels(self):
        return self.zero_order_channels + \
               self.first_order_channels + \
               self.second_order_channels

    def output_channel_name(self, channel: Tuple):
        assert len(channel) in {1, 2, 3}
        if len(channel) > 1:
            ch_names = [self.context.output_variables[channel[0]].nick_name] + \
                       [self.context.input_variables[i].nick_name for i in channel[1:]]
            return _node_derivative_prefix + _get_channel_name(tuple(ch_names))
        else:
            return self.context.output_variables[channel[0]].nick_name

    def output_channel_type(self, channel: Tuple):
        assert len(channel) > 0
        return self.context.output_variables[channel[0]].var_type


class FunctionBase:
    """


    """

    def __init__(self, input_spec: List[str], output_spec: List[str]):
        """
        Use '' to make function infer the type during call.
        :param output_spec: ['double','double']
        :param input_spec: ['double','float','']
        """

        assert all([(_is_valid_cpp_name(var_type) or var_type == '') for var_type in input_spec]), \
            "invalid input spec"
        assert all([(Variable.is_numerical_var_type(var_type) or var_type == '') for var_type in output_spec]), \
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

        existing_input_numerical_types = []
        for element in args:
            if type(element) is Variable:
                existing_input_numerical_types.append(element.var_type)
        inferred_var_type = Variable.infer_combined_var_type(existing_input_numerical_types)

        # Create input variables
        input_variables: List[Variable] = []
        for element in args:
            if type(element) is Variable:
                input_variables.append(element)
            elif type(element) in [float, int]:
                index = len(input_variables)
                var_type = inferred_var_type if self.input_spec[index] == '' else self.input_spec[index]
                assert var_type is not None, "Function with no numerical input can't have unspecified input type."

                unnamed_variable = graph.create_un_named_variable()
                unnamed_variable.defined_as_constant(element, var_type)
                input_variables.append(unnamed_variable)

        # Create output variables
        output_variables: List[Variable] = []
        for i in range(len(self.output_spec)):
            var_type = inferred_var_type if self.output_spec[i] == '' else self.output_spec[i]
            assert var_type is not None, "Function with no numerical input can't have unspecified output type."
            output_variable = graph.create_un_named_variable()
            output_variable.defined_as_expr(self, input_variables, i, var_type)
            output_variables.append(output_variable)

        # check the context compatibility
        context = Context(input_variables, output_variables)
        res, dbg = self.is_compatible(context)
        assert res, dbg

        # Inform graph the operation.
        graph.append_operation(self, context)

        if len(output_variables) > 1:
            return tuple(output_variables)
        else:
            return output_variables[0]

    def is_compatible(self, context: Context) -> Tuple[bool, str]:
        """
        :param context:
        :return: result, debug_string
        """
        out_dim = len(context.output_variables)
        in_dim = len(context.input_variables)

        if len(self.output_spec) != out_dim:
            return False, "output variable number miss-match"
        if len(self.input_spec) != in_dim:
            return False, "input variable number miss-match"

        # We check the type matching
        for i in range(out_dim):
            spec = self.output_spec[i]
            output_var_type = context.output_variables[i].var_type
            if spec == '':
                continue
            if spec == output_var_type or Variable.is_transferable(spec, output_var_type):
                continue
            return False, "The %d' th output type mis-match: require %s, got %s" % (i, spec, output_var_type)

        for i in range(in_dim):
            spec = self.input_spec[i]
            input_var_type = context.input_variables[i].var_type
            if spec == '':
                continue
            if spec == input_var_type or Variable.is_transferable(input_var_type, spec):
                continue
            return False, "The %d' th input type mis-match: require %s, got %s" % (i, spec, input_var_type)

        # check output TYPE to be EXPR
        for output_var in context.output_variables:
            if output_var.type not in [Variable.TYPE_STATE_EXPR, Variable.TYPE_CONFIG_EXPR]:
                return False, "output variables must be expr"
        return True, ""

    def print_definition(self) -> List[str]:
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

    def print_call(self, full_context: FullContext) -> Tuple[List[str], Dict[str, float]]:
        """

        result_names, result_types = context.get_result_names_and_types

        for example. result_names= ['ra','rb', 'NODE_Dra_Dx']

        Will:
        1, assume the inputs variables are defined and is ready.
        2, assume the output variables are defined yet not assigned with value.
        3, assume the non-constant output derivatives are defined yet not assigned with value.
        The function will print lines in C++ to calculate the output variables and non-constant derivatives.
        returned as List[str]
        The function will tell user which derivatives are constant and should not be defined.
        returned as Dist[str, float]

        Notice that user should call this function FIRST to know what to define.
        """
        assert False, "not implemented"


# the class that is ready to be

# Most functions should NOT provide constant outputs.
# this is because the interface/implementation would be dependent upon the context.

# so what we need is: context independent constant outputs.

# we need both context and function detail to determine whether an output der is constant.


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

    def is_compatible(self, context: Context) -> Tuple[bool, str]:
        res, dbg = super(SymPyFunction, self).is_compatible(context)

        if not res:
            return res, dbg

        if not all([Variable.is_numerical_var_type(input_var.var_type) for input_var in context.input_variables]):
            return False, "SympyFunction input variables must be numerical."

        if not all([Variable.is_numerical_var_type(output_var.var_type) for output_var in context.output_variables]):
            return False, "SympyFunction output variables must be numerical."

        return True, ""

    def print_call(self, full_context: FullContext) -> Tuple[List[str], Dict[str, float]]:

        res, dbg = self.is_compatible(full_context.context)
        assert res, dbg

        in_dim = len(full_context.context.input_variables)
        out_dim = len(full_context.context.output_variables)

        constant_results = {}

        # sympy environment
        # zero_order_output_exprs[i] corresponds to output_variable[i]
        # input_symbols[i] corresponds to input_variable[i]
        input_symbols = []
        for input_variable in full_context.context.input_variables:
            if input_variable.type is Variable.TYPE_CONSTANT:
                input_symbols.append(input_variable.value)
            else:
                input_symbols.append(sp.symbols(input_variable.nick_name))

        zero_order_output_exprs = self.sympy_function(*input_symbols)
        if len(full_context.context.output_variables) == 1:
            assert type(zero_order_output_exprs) is not tuple
            zero_order_output_exprs = [zero_order_output_exprs, ]
        else:
            assert type(zero_order_output_exprs) is tuple or type(zero_order_output_exprs) is list
        assert len(zero_order_output_exprs) == out_dim

        # Get the sympy expressions of each output.
        result_exprs = []
        result_names = []
        result_types = []

        output_channels = full_context.get_output_channels()

        for channel in output_channels:
            is_constant_derivative_output = False
            if len(channel) == 1:
                expr = zero_order_output_exprs[channel[0]]
            elif len(channel) == 2:
                expr = zero_order_output_exprs[channel[0]].diff(input_symbols[channel[1]])
                is_constant_derivative_output = expr.is_Number
            elif len(channel) == 3:
                expr = (zero_order_output_exprs[channel[0]].diff(input_symbols[channel[1]])).diff(
                    input_symbols[channel[2]])
                is_constant_derivative_output = expr.is_Number
            else:
                assert False

            res_name = full_context.output_channel_name(channel)
            res_type = full_context.output_channel_type(channel)

            # Arbitrate here/
            if is_constant_derivative_output:
                constant_results[res_name] = float(expr)
            else:
                result_exprs.append(expr)
                result_names.append(res_name)
                result_types.append(res_type)

        # Calculate the simplified expression.
        replacements, reduced_exprs = sp.cse(result_exprs, sp.utilities.iterables.numbered_symbols(_sympy_var_prefix))
        assert type(reduced_exprs) is tuple or type(reduced_exprs) is list
        assert len(result_exprs) == len(reduced_exprs)

        printer = OptimizedCXX11Printer()

        sympy_local_var_type = \
            Variable.infer_combined_var_type(
                [input_variable.var_type for input_variable in full_context.context.input_variables])

        assert sympy_local_var_type is not None

        lines = []
        # calculation steps
        lines.append("{")
        for replacement in replacements:
            this_line = printer.doprint(replacement[1], sympy_local_var_type + ' ' + str(replacement[0])).replace('\n',
                                                                                                                  '')
            lines.append(_indent + this_line)
        # assign outputs
        for i in range(len(reduced_exprs)):
            this_line = printer.doprint(reduced_exprs[i], result_names[i])
            lines.append(_indent + this_line)
        lines.append("}")

        return lines, constant_results


class _SymPyOperatorFunctions:
    sympy_add = SymPyFunction(lambda a, b: a + b)
    sympy_sub = SymPyFunction(lambda a, b: a - b)
    sympy_mul = SymPyFunction(lambda a, b: a * b)
    sympy_pow = SymPyFunction(lambda a, b: a ** b)
    sympy_div = SymPyFunction(lambda a, b: a / b)


"""
class CppMemberFunction(FunctionBase):
    def __init__(self, object_type: str, object_name: str, function_name: str, input_spec: List[str],
                 output_spec: List[str]):

        const CostSpace& space1;
        space1.collision_cost(...)
        :param object_type: CostSpace
        :param object_name: space1
        :param function_name: collision_cost
        # input , output types must be specified
        assert all([var_type in Variable.numerical_var_types for var_type in input_spec]), "invalid input spec"
        assert all([var_type in Variable.numerical_var_types for var_type in output_spec]), "invalid output spec"

        super(CppMemberFunction, self).__init__(input_spec, output_spec)

        assert _is_valid_namespace(object_type), "invalid object_type :(%s)" % object_type
        assert _is_valid_cpp_name(object_name), "invalid object_name :(%s)" % object_name
        assert _is_valid_cpp_name(function_name), "invalid function_name :(%s)" % function_name

        self.object_type = object_type
        self.object_name = object_name
        self.function_name = function_name
"""


# CppNamespaceFunction: header to include.
# What to have in the cpp file?
# the definition of each channel. I mean, at least you need constant outputs. implementation(assumed)
# there is a comment decoration protocol.
# 1, function name.
# 2, function constant outputs.
# 3, const references, just all what a Graph Function should have.
# 4, input spec.
# 5, output spec.

# interface function !! base class.


class ConditionalFunction(FunctionBase):
    # So we can avoid running a piece of code, by using gflag int.
    # We only support gflag int.
    # TODO(): implement with :
    #  1, checking no - loop dependency
    #  2, it is a little tricky, since each sub function are defining the output vars by themselves,
    #  There can be disagreement in the output var_types and even existence.
    #  That we need to merge them, their result types and names.
    pass


class GraphFieldManager:
    def __init__(self):
        self.existing_fields: Set[str] = set()
        self.constant_fields: Dict[str] = {}
        pass

    def claim_field_as_normal(self, name: str):
        self.existing_fields.add(name)

    def claim_field_as_constant(self, name: str, value: float):
        assert name not in self.constant_fields, "\'%s\' already claimed!" % name
        self.constant_fields[name] = value

    def is_constant(self, name: str):
        if name in self.existing_fields:
            return False
        else:
            return True

    def is_zero(self, name: str):
        if name in self.existing_fields:
            return False
        elif name in self.constant_fields:
            return self.constant_fields[name] == 0
        else:
            return True

    def get_constant_value(self, name: str):
        if name in self.constant_fields:
            return self.constant_fields[name]
        else:
            return 0

    def add_product_of_fields_to_target_field(self,
                                              output_lines: List[str],  # output
                                              target_field: str,
                                              target_field_type: str,
                                              fields_to_prod: List[str],
                                              gain: float = 1,
                                              indent_num: int = 0):
        if not fields_to_prod:
            return
        # at least adding something.

        constant_factor = gain
        for name in fields_to_prod:
            if self.is_constant(name):
                constant_factor *= self.get_constant_value(name)

        items = ''
        for name in fields_to_prod:
            if not self.is_constant(name):
                items += name + '*'
        if items != '':
            items = items[:-1]

        # items and constant factor append to field
        if constant_factor == 0:
            return

        all_indents = _indent * indent_num

        if items == '':
            # adding constant number
            if target_field in self.constant_fields:
                # add only to storage
                self.constant_fields[target_field] += constant_factor
                # output_lines.append('all_indents + // %s += %f;'%(target_field, constant_factor))
            elif target_field in self.existing_fields:
                # add to variable
                output_lines.append(all_indents + '%s += %f;' % (target_field, constant_factor))
            else:
                # create variable
                self.constant_fields[target_field] = constant_factor
                # output_lines.append('all_indents + // %s = %f;'%(target_field, constant_factor))
        else:
            # adding expression
            expr = "(%f) * %s" % (constant_factor, items) if constant_factor != 1 else items
            if target_field in self.constant_fields:
                # no longer constant, become existing normal
                value = self.constant_fields[target_field]
                full_expr = '%f + %s' % (value, items) if value != 0 else items
                output_lines.append(all_indents + '%s %s=%s;' % (target_field_type, target_field, full_expr))

                del self.constant_fields[target_field]
                self.existing_fields.add(target_field)
            elif target_field in self.existing_fields:
                # add expr to existing field.
                output_lines.append(all_indents + '%s += %s;' % (target_field, expr))
            else:
                # create existing field with expr
                output_lines.append(all_indents + '%s %s=%s;' % (target_field_type, target_field, expr))
                self.existing_fields.add(target_field)


class InterfaceFunction(FunctionBase):
    def __init__(self, input_spec: List[str],
                 output_spec: List[str],
                 function_name: str,
                 constant_derivative_outputs: Dict[Tuple, float]):
        # 1, a specification of which dout_din is always constant and should be implementation wise recorded.
        # when print call, 2 types of contantification:
        # 1, context irrelevant constant
        # 2, context relevant constant (like putting constant to stateful channel, and thusu some interfaces not used.)

        # Make sure the field constant_outputs only consider 1, and make sure print_call lines are considering 2.
        super(InterfaceFunction, self).__init__(input_spec, output_spec)

        self.constant_derivative_outputs = constant_derivative_outputs
        self.function_name = function_name

    # It is a trick
    def calling_prefix(self):
        return ''

    def print_call(self, full_context: FullContext) -> Tuple[List[str], Dict[str, float]]:
        in_dim = len(full_context.context.input_variables)
        out_dim = len(full_context.context.output_variables)

        # determine the interface
        full_channels = _full_output_channels_with_derivatives(in_dim, out_dim,
                                                               full_context.option.enable_1st_order_derivative,
                                                               full_context.option.enable_2nd_order_derivative)
        interface_output_channels = []
        for channel in full_channels:
            if channel not in self.constant_derivative_outputs:
                interface_output_channels.append(channel)

        channels_required = set(full_context.get_output_channels())
        interface_output_names = []
        # type to name
        unused_variables: Dict[str, str] = {}
        for channel in interface_output_channels:
            if channel in channels_required:
                interface_output_names.append(full_context.output_channel_name(channel))
            else:
                var_type = full_context.output_channel_type(channel)
                if var_type not in unused_variables:
                    unused_variables[var_type] = _graph_unused_prefix + var_type
                interface_output_names.append(unused_variables[var_type])

        lines = []
        lines.append("{")
        for var_type, var_name in unused_variables.items():
            lines.append(_indent + var_type + " " + var_name + ";")

        calling = self.calling_prefix()
        calling += full_context.option.decorate(self.function_name)

        calling += "("

        for i in range(in_dim):
            calling += full_context.context.input_variables[i].nick_name + ","

        for out_name in interface_output_names:
            calling += "&" + out_name + ","

        if calling[-1] == ",":
            calling = calling[:-1]
        calling += ");"

        lines.append(calling)

        lines.append("}")

        # constant_derivative_outputs.
        constant_derivative_outputs = {}
        for channel, value in self.constant_derivative_outputs.items():
            constant_derivative_outputs[full_context.output_channel_name(channel)] = value

        return lines, constant_derivative_outputs

    def comment_header(self):
        return '=====InterfaceSpec[%s]=====' % self.function_name

    @staticmethod
    def channel2str(channel):
        assert len(channel) in {1, 2, 3}
        return "_".join([str(ch) for ch in channel])

    @staticmethod
    def str2channel(string: str)->Tuple:
        return tuple([int(ch_str) for ch_str in str.split("_")])

    def to_comments(self) -> List[str]:
        comment = []
        comment.append("input_spec:" + ','.join(self.input_spec))
        comment.append("output_spec:" + ','.join(self.output_spec))

        const_channels = []
        const_values = []
        for channel, value in self.constant_derivative_outputs.items():
            const_channels.append(str(channel))
            const_values.append(str(value))

        comment.append("const_channels:" + ','.join(const_channels))
        comment.append("const_values:" + ','.join(const_values))
        return comment

    def load_interface_from_comment(self, comments: List[str]):
        """
        :param comments: contents between 2 comment_header, shouldn't have '\n' or '\r'
        :return: change class member in place
        """
        debug_prefix = "load_interface_from_comment:"
        # Store each list by field name
        fields = {}
        for comment_line in comments:
            colon = comment_line.find(":")
            if colon < -1:
                continue
            field_name = comment_line[:colon].replace(' ', '')
            field_elements = comment_line[colon + 1:].replace(' ', '')

            if field_name in fields:
                assert False, debug_prefix + "repeated field in comments: %s" % field_name
            fields[field_name] = field_elements

        # input_spec
        assert "input_spec" in fields, debug_prefix + "comments must have input_spec"
        self.input_spec = fields["input_spec"].split(',')
        assert all([_is_valid_cpp_name(var_type) for var_type in self.input_spec]), debug_prefix + "invalid input spec"

        # output_spec
        assert "output_spec" in fields, debug_prefix + "comments must have output_spec"
        self.output_spec = fields["output_spec"].split(',')
        assert all([Variable.is_numerical_var_type(var_type) for var_type in self.output_spec]), \
            debug_prefix + "invalid output spec"

        # constant_derivative_outputs
        assert "const_channels" in fields, debug_prefix + "comments must have const_channels"
        assert "const_values" in fields, debug_prefix + "comments must have const_values"
        try:
            const_channels = [self.str2channel(word) for word in fields["const_channels"].split(',')]
            const_values = [float(word) for word in fields["const_values"].split(',')]
        except:
            assert False, debug_prefix + "failed to parse const_channels or const_values"

        assert len(const_channels) == \
               len(const_values), debug_prefix + "const_values and const_channels must have same length"

        self.constant_derivative_outputs = {}
        for i in range(len(const_values)):
            self.constant_derivative_outputs[const_channels[i]] = const_values[i]


class GraphFunction(InterfaceFunction):
    #  TODO(): implement with :
    #  1, checking no - loop dependency
    #  2, assert each outer graph statefuls are not passed as config to this graph.
    def __init__(self,
                 input_spec: List[str],
                 output_spec: List[str],
                 function_name: str,
                 constant_derivative_outputs: Dict[Tuple, float],
                 implementations: List[List[str]]):
        super(GraphFunction, self).__init__(input_spec, output_spec,
                                            function_name, constant_derivative_outputs)

        # implementation of each option in _all_options.
        assert len(implementations) == len(_all_options)
        self.implementations = implementations

    def print_definition(self) -> List[str]:
        result = []

        for implementation in self.implementations:
            result += implementation
            result += ['']
        return result

    def dump_to_cpp_file(self, filename:  str, namespaces: List[str]):
        pass


class CppFunction(FunctionBase):
    # TODO(huaiyuan): Graph function load-able.
    # if some graph uses me, its generated cpp must also depends on my header.
    # here the dirty work begins.
    def __init__(self, header_filename: str, function_name: str, class_name: str):
        pass


class Graph:
    def __init__(self, name: str = "UntitledFunc"):
        # contain all variables
        # from nick_name to Variable
        self.name = name
        self._all_variables: Dict[str, Variable] = {}

        self._un_named_count = 0

        self._operations: List[Tuple[FunctionBase, Context]] = []

    def state_inputs(self, names: List[str], var_type: str):
        assert Variable.is_numerical_var_type(var_type), "Invalid type: %s" % var_type
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
        assert _is_valid_cpp_name(var_type), "Invalid type: %s" % var_type
        if not names:
            return None

        results = [self.create_new_variable(name) for name in names]
        for var in results:
            var.defined_as_config_input(var_type)

        if len(results) == 1:
            return results[0]
        else:
            return results

    def append_operation(self, function: FunctionBase, context: Context):
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

    # TODO(huaiyuan): may be this is a member function of GraphFunction. Just Maybe
    # TODO(huaiyuan): comment on implementation: how many addition, how many multiplication
    def print_derivative_definition(self,
                                    input_variables: List[Variable],
                                    output_variables: List[Variable],
                                    option: Option) -> Tuple[List[str], Dict[Tuple, float]]:
        """
        :param input_variables: you will use this for ordering, although the graph know what it takes
        :param output_variables: a list of output variables, must be member of the graph
        :param option: specifying the generation option
        :return: (implementation with function head, Dict of constant derivative outputs)
        """
        assert option in _all_options, "option Must be one of _all_options"

        # 1, you cannot determine interface until you
        # once determined input variables and output variables, we can get the derivative order of them, their
        # BP core:
        # 1, set of declared vars. check existence when using. special simplification when absence
        # 2, when creating new declared vars, check existence for addition.

        for variable in output_variables:
            assert self.is_member(variable), "output variables must be member of the graph."

        for variable in input_variables:
            assert self.is_member(variable), "input variables must be member of the graph."

        names_of_inputs = set()
        for variable in self._all_variables.values():
            if variable.type in {Variable.TYPE_STATE_INPUT, Variable.TYPE_CONFIG_INPUT}:
                names_of_inputs.add(variable.nick_name)

        assert len(input_variables) == len(names_of_inputs), \
            "please make sure input_variables exactly contains: " + str(names_of_inputs)
        assert names_of_inputs == {variable.nick_name for variable in input_variables}, \
            "please make sure input_variables exactly contains: " + str(names_of_inputs)

        # when asked 2nd order derivative, 1st order must be also ready.
        sub_function_option = Option(
            enable_1st_order_derivative=option.enable_2nd_order_derivative or option.enable_1st_order_derivative,
            enable_2nd_order_derivative=option.enable_2nd_order_derivative)
        assert sub_function_option in _all_options, "sub option Must be one of _all_options"

        lines = []
        manager = GraphFieldManager()

        for function, context in self._operations:
            full_context = FullContext(context,
                                       sub_function_option)

            calculation, constant_outputs = \
                function.print_call(full_context)
            # TODO(): clear unused variables making use of AC automaton.

            channels = full_context.get_output_channels()
            for channel in channels:
                result_name = full_context.output_channel_name(channel)
                result_type = full_context.output_channel_type(channel)
                if result_name not in constant_outputs:
                    lines.append(_indent + result_type + ' ' + result_name + ';')
                    manager.claim_field_as_normal(result_name)
                else:
                    manager.claim_field_as_constant(result_name, constant_outputs[result_name])

            lines += [_indent + ln for ln in calculation]

        def get_graph_derivative_name(out_name: str, in_names: List[str]):
            in_names_copy = in_names.copy()
            in_names_copy.sort()

            order = len(in_names_copy)
            assert order in [1, 2]

            if order is 1:
                return _graph_derivative_prefix + _get_channel_name((out_name, in_names_copy[0]))
            elif order is 2:
                return _graph_derivative_prefix + _get_channel_name((out_name, *in_names_copy))
            else:
                assert False

        if option.enable_1st_order_derivative or option.enable_2nd_order_derivative:
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

                # starts from the fact da_da = 1.
                manager.claim_field_as_constant(
                    get_graph_derivative_name(active_variable.nick_name, [active_variable.nick_name, ]), 1)

                # bp the entire graph
                for i in range(step, -1, -1):
                    _, context = self._operations[i]
                    full_context = FullContext(context,
                                               sub_function_option)

                    # Update graph derivative from Node derivatives
                    for out_idx, in_idx in full_context.first_order_channels:
                        node_der_name = full_context.output_channel_name((out_idx, in_idx))
                        in_name = context.input_variables[in_idx].nick_name
                        out_name = context.output_variables[out_idx].nick_name

                        # Simple chain rule
                        d_active_d_node_in = get_graph_derivative_name(active_variable.nick_name, [in_name, ])
                        d_active_d_node_out = get_graph_derivative_name(active_variable.nick_name, [out_name, ])

                        manager.add_product_of_fields_to_target_field(lines,
                                                                      d_active_d_node_in, active_var_type,
                                                                      [d_active_d_node_out, node_der_name],
                                                                      indent_num=1)

        if option.enable_2nd_order_derivative:
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
                    full_context = FullContext(context,
                                               sub_function_option)

                    # Update graph derivative from Node derivatives
                    for out_idx, in_idx_1, in_idx_2 in full_context.second_order_channels:
                        in_1_name = full_context.context.input_variables[in_idx_1].nick_name
                        in_2_name = full_context.context.input_variables[in_idx_2].nick_name
                        out_name = full_context.context.output_variables[out_idx].nick_name

                        d2_active_d_node_in_1_d_node_in_2 = \
                            get_graph_derivative_name(active_variable.nick_name, [in_1_name, in_2_name])
                        d_active_d_node_out = get_graph_derivative_name(active_variable.nick_name, [out_name, ])
                        d2_active_d_node_out_d_node_out = \
                            get_graph_derivative_name(active_variable.nick_name, [out_name, out_name])

                        node_d2_out_d_in_1_d_in_2 = full_context.output_channel_name((out_idx, in_idx_1, in_idx_2))
                        node_d_out_d_in_1 = full_context.output_channel_name((out_idx, in_idx_1))
                        node_d_out_d_in_2 = full_context.output_channel_name((out_idx, in_idx_2))

                        manager.add_product_of_fields_to_target_field(lines,
                                                                      d2_active_d_node_in_1_d_node_in_2,
                                                                      active_var_type,
                                                                      [d_active_d_node_out, node_d2_out_d_in_1_d_in_2],
                                                                      indent_num=1)

                        manager.add_product_of_fields_to_target_field(lines,
                                                                      d2_active_d_node_in_1_d_node_in_2,
                                                                      active_var_type,
                                                                      [d2_active_d_node_out_d_node_out,
                                                                       node_d_out_d_in_1, node_d_out_d_in_2],
                                                                      indent_num=1)

                        if in_idx_1 != in_idx_2 and \
                                not manager.is_zero(d2_active_d_node_in_1_d_node_in_2):
                            co_relate(in_1_name, in_2_name, existing_cross_items_of_variable)

                    for out_idx, in_idx in full_context.first_order_channels:
                        in_name = full_context.context.input_variables[in_idx].nick_name
                        out_name = full_context.context.output_variables[out_idx].nick_name

                        # output channel has no co-related items
                        if out_name not in existing_cross_items_of_variable:
                            continue

                        node_d_out_d_in = full_context.output_channel_name((out_idx, in_idx))
                        for co_related in existing_cross_items_of_variable[out_name]:
                            d2_active_d_node_out_d_co_related = \
                                get_graph_derivative_name(active_variable.nick_name, [out_name, co_related])
                            d2_active_d_node_in_d_co_related = \
                                get_graph_derivative_name(active_variable.nick_name, [in_name, co_related])
                            if co_related != in_name:
                                manager.add_product_of_fields_to_target_field(lines,
                                                                              d2_active_d_node_in_d_co_related,
                                                                              active_var_type,
                                                                              [d2_active_d_node_out_d_co_related,
                                                                               node_d_out_d_in],
                                                                              indent_num=1)
                            else:
                                manager.add_product_of_fields_to_target_field(lines,
                                                                              d2_active_d_node_in_d_co_related,
                                                                              active_var_type,
                                                                              [d2_active_d_node_out_d_co_related,
                                                                               node_d_out_d_in],
                                                                              indent_num=1,
                                                                              gain=2)

                            if co_related != in_name and \
                                    not manager.is_zero(d2_active_d_node_in_d_co_related):
                                co_relate(in_name, co_related, existing_cross_items_of_variable)

        # Figure out which derivative output channel has been silenced
        # 2 ways of silenced: it is not differentiable, it is zeroed.
        constant_derivative_outputs: Dict[Tuple, float] = {}

        full_output_channels = _full_output_channels_with_derivatives(len(input_variables), len(output_variables),
                                                                      option.enable_1st_order_derivative,
                                                                      option.enable_2nd_order_derivative)

        def graph_name_of_output_channel(out_channel: Tuple):
            assert len(out_channel) in {1, 2, 3}
            if len(out_channel) == 1:
                return output_variables[out_channel[0]].nick_name
            else:
                return get_graph_derivative_name(
                    output_variables[out_channel[0]].nick_name,
                    [input_variables[in_idx].nick_name for in_idx in out_channel[1:]])

        for channel in full_output_channels:
            assert len(channel) in {1, 2, 3}
            if len(channel) > 1:
                fully_differentiable = \
                    output_variables[channel[0]].is_differentiable() and all(
                        [input_variables[in_idx].is_differentiable() for in_idx in channel[1:]])

                if not fully_differentiable:
                    constant_derivative_outputs[channel] = 0
                    continue

                graph_name = graph_name_of_output_channel(channel)
                if manager.is_constant(graph_name):
                    constant_derivative_outputs[channel] = manager.get_constant_value(graph_name)

        # Determine the function head
        # TODO( fill out const references)
        const_references = []

        function_head = 'void ' + option.decorate(self.name) + "("

        # inputs:
        for input_var in input_variables:
            interface_name = input_var.nick_name
            interface_type = input_var.var_type
            comment = "/*state*/" if input_var.is_differentiable() else "/*config*/"
            function_head += Variable.const_reference(interface_type) + ' ' + interface_name + comment + ','

        # outputs:
        full_output_channels = _full_output_channels_with_derivatives(len(input_variables), len(output_variables),
                                                                      option.enable_1st_order_derivative,
                                                                      option.enable_2nd_order_derivative)

        for channel in full_output_channels:
            if channel in constant_derivative_outputs:
                continue
            graph_name = graph_name_of_output_channel(channel)
            interface_name = _graph_output_prefix + graph_name
            interface_type = output_variables[channel[0]].var_type

            function_head += interface_type + "* " + interface_name + ","
            lines.append(_indent + '*' + interface_name + '=' +
                         graph_name + ';')

        if function_head[-1] == ',':
            function_head = function_head[:-1]
        function_head += ") {"
        lines.append("}")

        return [function_head] + lines, constant_derivative_outputs

    def create_graph_function(self, input_variables: List[Variable],
                              output_variables: List[Variable],
                              name: str = None):
        if name is None:
            name = self.name
        assert _is_valid_cpp_name(name)

        input_spec = [input_var.var_type for input_var in input_variables]
        output_spec = [output_var.var_type for output_var in output_variables]

        implementations = []
        constant_derivative_outputs = {}

        for option in _all_options:
            implementation, const_der_out = self.print_derivative_definition(input_variables, output_variables, option)
            implementations.append(implementation)
            for channel, value in const_der_out.items():
                # Skip different option's consistency check
                constant_derivative_outputs[channel] = value

        return GraphFunction(input_spec, output_spec, name, constant_derivative_outputs,
                             implementations)

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
c = g.config_inputs(['c'], 'Gref')
radius = x ** 2 + (y ** 2)

radius.set_name('r')

create_rad = g.create_graph_function([x, c, y], [radius], 'ComputeRad')

lines = create_rad.print_definition()
s = ''
for line in lines:
    s += line
    s += '\n'
print(s)

# TODO(): test validity in C++ project. operators. Bp process for gradients. (single output)
