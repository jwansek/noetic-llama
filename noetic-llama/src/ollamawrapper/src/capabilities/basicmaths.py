import rospy


def add(num1, num2):
    """Returns the output of two numbers added together

    Args:
        num1 (int): The first number to add together
        num2 (int): The second number to add together

    Returns:
        int: The sum of the two numbers
    """
    print("%f + %f = %f" % (num1, num2, num1 + num2))
    return num1 + num2

def sub(num1, num2):
    """Returns the output of two numbers subtracted from each other

    Args:
        num1 (int): The first number
        num2 (int): A number to subtract the first number from
    """
    print("%f - %f = %f" % (num1, num2, num1 - num2))
    return num1 - num2


def mult(num1, num2):
    """Returns the output of two numbers multiplied with each other

    Args:
        num1 (int): The first number to multiply together
        num2 (int): The second number to multiply together
    """
    print("%f × %f = %f" % (num1, num2, num1 * num2))
    return  num1 * num2

def div(num1, num2):
    """Returns the output of two numbers divided with each other

    Args:
        num1 (int): The enumerator
        num2 (int): The denominator
    """
    print("%f ÷ %f = %f" % (num1, num2, num1 / num2))
    return num1 / num2

"""class A:
    def funct():


a = A()

def func():
    a.funct()"""
