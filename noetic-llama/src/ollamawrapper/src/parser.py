"""This is a WIP that parses the ollama output with the idea being to call
functions using the python API instead of using `exec()` which is rather unsafe
Since the structure is rather complicated it requires defining a grammar to do 
parsing with.

The grammar works but navigating the abstract syntax tree requires some work.
"""
from parsimonious.grammar import Grammar
from parsimonious.nodes import NodeVisitor

class FunctionCaller(NodeVisitor):

    cur_args = {}

    def visit_functioncall(self, node, visited_children):
        """ Gets each key/value pair, returns a tuple. """
        print("***functioncall***", node.text, self.cur_args, "sneed")
        self.cur_args = {}
        # key, _, value, *_ = node.children
        # return key.text, value.text

    def visit_argdecl(self, node, visited_children):
        """ Gets each key/value pair, returns a tuple. """
        key, _, value, *_ = node.children
        self.cur_args[key.text] = value.text
        return {key.text: value.text}

    def generic_visit(self, node, visited_children):
        """ The generic visit method. """
        return visited_children or node

with open("ollamafunctiongrammar.ppeg", "r") as f:
    grammar = Grammar(f.read())

tree = grammar.parse("get_weather_data(coordinates=get_coordinates_from_city(city_name='Lincoln'), add=sum(num1=2, num2=3))")
iv = FunctionCaller()
print(iv.visit(tree))