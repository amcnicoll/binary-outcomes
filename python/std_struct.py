"""
We frequently use packed structs for serial protocols.
Python's built-in ctypes module does a good job of fitting the bill.
This module defines a "standard struct" class which addresses shortcomings:

- Based off ctypes.LittleEndianStructure to match our architecture

- Packed by default

- Standard way to serialize to bytearray is with bytearray(struct).
  This breaks down for variable-length structs, so we standardize
  on a struct.serialize() method across our codebase.

- Similarly, structs with a variable-length field at the end are usually
  instantiated from buffers smaller than the struct allocation. Since
  from_buffer_copy complains about this, we add from_short_buffer.

- __str__ is actually useful by default, rather than just a __repr__
  It provides a C-def style indented representation of the struct
  This also comes with _string_include_ and _string_exclude_ attributes,
  which are lists of attributes to include and field names to exclude,
  respectively. Useful for things like bitfields or pad fields.

- A "compact" version of __str__, compact_string(), which goes for a one-liner

- _enum_lookup_, and optional dictionary attribute which is a hint for uint8
  fields that should be displayed as IntEnums (our equivalent for C enums)

- Helper function assign_array for flexibly copying Python bytearrays,
  lists, strings, etc. into a ctypes array.

"""
import ctypes
from util import to_bytearray
from util import indent


class StandardStruct(ctypes.LittleEndianStructure):

    _pack_              = 1

    _enum_lookup_       = []

    _string_include_    = []
    _string_exclude_    = []

    def __str__(self):
        ret = self.__class__.__name__ + ' struct {\n'
        for name, type in self._fields_:
            if name in self._string_exclude_:
                continue
            attr = getattr(self, name)
            if name in self._enum_lookup_:
                member_string = self.get_enum_name(name)
            if name not in self._string_exclude_:
                member_string = str(attr)
                # Arrays are left as-is for now
                # We may want to adaptively convert to string later
                # If this is a single nested struct, indent
                if isinstance(attr, StandardStruct):
                    member_string = indent(member_string, leading_char='')
            ret += '\t%-16s= %s\n' % (name, member_string)
        for name in self._string_include_:
            attr = getattr(self, name)
            member_string = str(attr)
            ret += '\t%-16s= %s\n' % ('~ ' + name, member_string)
        ret += '}'
        return ret

    def __repr__(self):
        return '<' + self.__class__.__name__ + '>'

    def get_enum_name(self, field):
        klass = self._enum_lookup_[field]
        try:
            good_enum = klass(getattr(self, field))
            return str(good_enum)
        except ValueError:
            return klass.__name__ + '.UNKNOWN'

    def compact_string(self):
        ret = self.__class__.__name__ + ':'
        for name, type in self._fields_:
            if name not in self._string_exclude_:
                if name in self._string_exclude_:
                    continue
                attr = getattr(self, name)
                if name in self._enum_lookup_:
                    member_string = self.get_enum_name(name)
                if name not in self._string_exclude_:
                    member_string = str(attr)
                if isinstance(attr, StandardStruct):
                    member_string = attr.__repr__()
                ret += ' %s %s,' % (name, member_string)
        for name in self._string_include_:
            attr = getattr(self, name)
            member_string = str(attr)
            ret += ' %s %s,' % ('~' + name, member_string)
        return ret[:-1]

    def serialize(self):
        return bytearray(self)

    def assign_integer_array(self, field, py_integer_list, pad_val=0):
        cval  = getattr(self, field)
        args  = py_integer_list[:len(cval)]
        args += [pad_val] * (len(cval) - len(args))
        setattr(self, field, cval.__class__(*args))

    def assign_byte_array(self, field, py_bytearray_castable, pad_val=0):
        as_byte = to_bytearray(py_bytearray_castable)
        as_list = [x for x in as_byte]
        self.assign_integer_array(field, as_list, pad_val=pad_val)

    @classmethod
    def from_address_of(cls, ctype_obj):
        return cls.from_address(ctypes.addressof(ctype_obj))

    @classmethod
    def from_short_buffer(cls, buff, pad_char='\0'):
        buff = buff.ljust(ctypes.sizeof(cls), pad_char)
        return cls.from_buffer_copy(buff)
