# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from typing import Callable, List


def assert_not_errors(errors: List) -> None:
    assert not errors, 'Errors: %s' % '\n'.join(errors)


def valid_object_variable_check(
    valid_entries: List, init_test: Callable, set_test: Callable, get_func: Callable
) -> List:
    errors = []
    for entry in valid_entries:
        # Initialization Test
        try:
            obj = init_test(entry)
        except AssertionError as e:
            errors.append(
                'Valid entry %s was incorrectly rejected with the following message: %s'
                % (entry, e.args[0])
            )
            continue
        var = get_func(obj)
        if var != type(var)(entry):
            errors.append('Valid variable %s was incorrectly set as %s' % (entry, var))
        # Set Test
        try:
            obj = init_test(entry)
            set_test(obj, entry)
        except AssertionError as e:
            errors.append(
                'Valid entry %s was incorrectly rejected with the following message: %s'
                % (entry, e.args[0])
            )
            continue
        var = get_func(obj)
        if var != type(var)(entry):
            errors.append('Valid variable %s was incorrectly set as %s' % (entry, var))
    return errors


def invalid_object_variable_check(
    invalid_entries: List, init_test: Callable, set_test: Callable
) -> List:
    errors = []
    for entry in invalid_entries:
        # Initialization Test
        try:
            obj = init_test(entry)
        except AssertionError:
            pass  # Do nothing, expected
        else:
            errors.append('Invalid variable %s was incorrectly accepted' % (entry))
        # Set Test
        try:
            obj = init_test(entry)
            set_test(obj, entry)
        except AssertionError:
            pass  # Do nothing, expected
        else:
            errors.append('Invalid variable %s was incorrectly accepted' % (entry))
    return errors
