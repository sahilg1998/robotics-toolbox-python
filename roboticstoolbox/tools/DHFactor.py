"""
@author Samuel Drew
"""

import re

class Element:

    TX = 0
    TY = 1
    TZ = 2
    RX = 3
    RY = 4
    RZ = 5
    DH_STANDARD = 6
    DH_MODIFIED = 7

    # an array of counters for the application of each rule
    # just for debugging.
    rules = [0] * 20

    # mapping from type to string
    typeName = ["TX", "TY", "TZ", "RX", "RY", "RZ", "DH", "DHm"]

    # order of elementary transform for each DH convention
    # in each tuple, the first element is the transform type,
    # the second is true if it can be a joint variable.
    dhStandard = (RZ, 1), (TX, 0), (TZ, 1), (RX, 0)
    dhModified = (RX, 0), (TX, 0), (RZ, 1), (TZ, 1)


    def __init__(
            self,
            elementIn=None,
            stringIn=None,
            eltype=None,
            constant=None,
            sign=0):

        self.var = None         # eg. q1, for joint var types
        self.symconst = None    # eg. L1, for lengths

        # DH parameters, only set if type is DH_STANDARD/MODIFIED
        self.theta = None
        self.alpha = None
        self.A = None
        self.D = None
        self.prismatic = None
        self.offset = None
        self.constant = None

        if stringIn:
            if elementIn or eltype or constant:
                raise ValueError("if parsing a string, string must be the only input")
            i = None
            sType = stringIn[0:2]     # Tx, Rx etc
            sRest = stringIn[2:]       # the argument including brackets

            if not (sRest[-1] == ")" and sRest[0] == "("):
                raise ValueError("brackets")

            match = False
            for i in range(6):
                check = self.typeName[i].lower()
                if sType.lower() == check:
                    match = True
                    self.eltype = i
            if not match:
                raise ValueError("bad transform name: " + sType)

            sRest = sRest[1:-1]     # get the argument from between brackets

            # handle an optional minus sign
            negative = ""

            if sRest[0] == '-':
                negative = "-"
                sRest = sRest[1]

            if sRest[0] == "q":
                self.var = negative + sRest
            elif sRest[0] == 'L':
                self.symconst = negative + sRest
            else:
                try:
                    self.constant = float(sRest)
                    if negative == "-":
                        self.constant = -self.constant
                        negative = ""
                except:
                    raise ValueError("bad argument in term " + stringIn)

        elif elementIn:
            if not isinstance(elementIn, Element):
                raise TypeError("elementIn must be an existing Element object")
            self.eltype = elementIn.eltype
            if elementIn.var:
                self.var = elementIn.var
            if elementIn.symconst:
                self.symconst = elementIn.symconst
            self.constant = elementIn.constant
            # if sign < 0:
            #     self.negate()

        # one of TX, TY ... RZ, DH_STANDARD/MODIFIED
        if eltype:
            self.eltype = eltype
            if constant:
                self.constant = constant    # eg. 90, for angles
            if sign < 0:
                self.negate()

    def __str__(self):
        return self.toString()

    @staticmethod
    def showRuleUsage():
        for i in range(20):
            if Element.rules[i] > 0:
                print("Rule " + str(i) + ": " + str(Element.rules[i]))

    def istrans(self):
        return (self.eltype == self.TX) or (self.eltype == self.TY) or (self.eltype == self.TZ)

    def isrot(self):
        return (self.eltype == self.RX) or (self.eltype == self.RY) or (self.eltype == self.RZ)

    def isjoint(self):
        return self.var is not None

    def axis(self):
        if self.eltype == self.RX or self.eltype == self.TX:
            return 0
        elif self.eltype == self.RY or self.eltype == self.TY:
            return 1
        elif self.eltype == self.RZ or self.eltype == self.TZ:
            return 2
        else:
            raise ValueError("bad transform type")

    def symAdd(self, s1, s2):

        if s1 is None and s2 is None:
            return None
        elif s1 and s2 is None:
            return s1
        elif s1 is None and s2:
            return s2
        else:
            if s2[0] == "-":
                return s1 + s2
            else:
                return s1 + "+" + s2

    def add(self, e):
        if self.eltype != Element.DH_STANDARD and self.eltype != Element.DH_MODIFIED:
            raise ValueError("wrong element type " + str(self))
        print("  adding: " + str(self) + " += " + str(e))
        if e.eltype == self.RZ:
            if e.isjoint():
                self.prismatic = 0
                self.var = e.var
                self.offset = e.constant
                self.theta = 0.0
            else:
                self.theta = e.constant
        elif e.eltype == self.TX:
            self.A = e.symconst
        elif e.eltype == self.TZ:
            if e.isjoint():
                self.prismatic = 1
                self.var = e.var
                self.D = None
            else:
                self.D = e.symconst
        elif e.eltype == self.RX:
            self.alpha = e.constant
        else:
            raise ValueError("Can't factorise " + str(e))

    # test if this particular element could be part of a DH term
    # eg. Rz(q1) can be, Rx(q1) cannot.
    def factorMatch(self, dhWhich, i, verbose):

        dhFactors = None
        match = False

        if dhWhich == self.DH_STANDARD:
            dhFactors = self.dhStandard
        elif dhWhich == self.DH_MODIFIED:
            dhFactors = self.dhModified
        else:
            raise ValueError("bad DH type")

        match =	(self.eltype == dhFactors[i][0]) and not ((dhFactors[i][1] == 0) and self.isjoint())

        if verbose > 0:
            print(" matching " + str(self) + " (i=" + str(i) + ") " +
                  " to " + self.typeName[dhFactors[i][0]] + "<" +
                  str(dhFactors[i][1]) + ">" + " -> " + str(match))
        return match


    def merge(self, e):
        assert type(e) == Element, "merge(Element e)"
        """
        don't merge if dissimilar transform or
        both are joint variables
        """
        if e.eltype != self.eltype or e.isjoint() and self.isjoint():
            return self

        sum = Element(self)

        sum.var = self.symAdd(self.var, e.var)
        sum.symconst = self.symAdd(self.symconst, e.symconst)
        sum.constant = self.constant + e.constant

        if not sum.isjoint() and sum.symconst is None and sum.constant == 0:
            print("Eliminate: " + self + " " + e)
            return None
        else:
            print("Merge: " + self + " " + e + " := " + sum)
            return sum

    '''
    test if two transforms need  to be swapped
    @param e	the element to compare with this
    @return	- true if swap is required
    '''
    def swap(self, next, dhWhich):
        assert type(next) == Element, "type(next) == Element"

        # don't swap if both are joint variables
        if self.isjoint() and next.isjoint():
            return False

        if dhWhich == Element.DH_STANDARD:
            order = [2, 0, 3, 4, 0, 1]
            if self.eltype == Element.TZ and next.eltype == Element.TX or \
                    self.eltype == Element.TX and next.eltype == Element.RX and next.isjoint() or \
                    self.eltype == Element.TY and next.eltype == Element.RY and next.isjoint() or \
                    self.eltype == Element.TZ and next.eltype == Element.RZ and next.isjoint() or \
                    not self.isjoint() and self.eltype == Element.RX and next.eltype == Element.TX or \
                    not self.isjoint() and self.eltype == Element.RY and next.eltype == Element.TY or \
                    not self.isjoint() and not next.isjoint() and self.eltype == Element.TZ and next.eltype == Element.RZ or \
                    self.eltype == Element.TY and next.eltype == Element.TZ or \
                    self.eltype == Element.TY and next.eltype == Element.TX:
                print("Swap: " + self + " <-> " + next)
                return True
        elif dhWhich == Element.DH_MODIFIED:
            if self.eltype == Element.RX and next.eltype == Element.TX or \
                    self.eltype == Element.RY and next.eltype == Element.TY or \
                    self.eltype == Element.RZ and next.eltype == Element.TZ or \
                    self.eltype == Element.TZ and next.eltype == Element.TX:
                print("Swap: " + self + " <-> " + next)
                return True
        else:
            raise ValueError("bad DH type")
        return False

    def substituteToZ(self):
        s = list()

        if self.eltype == Element.RX:
            s.append(Element(eltype=Element.RY, constant=90.0))
            s.append(Element(elementIn=self, eltype=Element.RZ))
            s.append(Element(eltype=Element.RY, constant=-90.0))
            return s
        elif self.eltype == Element.RY:
            s.append(Element(eltype=Element.RX, constant=-90.0))
            s.append(Element(elementIn=self, eltype=Element.RZ))
            s.append(Element(eltype=Element.RX, constant=90.0))
            return s
        elif self.eltype == Element.TX:
            s.append(Element(eltype=Element.RY, constant=90.0))
            s.append(Element(elementIn=self, eltype=Element.TZ))
            s.append(Element(eltype=Element.RY, constant=-90.0))
            return s
        elif self.eltype == Element.TY:
            s.append(Element(eltype=Element.RX, constant=-90.0))
            s.append(Element(elementIn=self, eltype=Element.TZ))
            s.append(Element(eltype=Element.RX, constant=90.0))
            return s
        else:
            return None

    def substituteToZ(self, prev=None):
        s = list()

        if not prev:
            if self.eltype == Element.RX:
                s.append(Element(eltype=Element.RY, constant=90.0))
                s.append(Element(elementIn=self, eltype=Element.RZ))
                s.append(Element(eltype=Element.RY, constant=-90.0))
                return s
            elif self.eltype == Element.RY:
                s.append(Element(eltype=Element.RX, constant=-90.0))
                s.append(Element(elementIn=self, eltype=Element.RZ))
                s.append(Element(eltype=Element.RX, constant=90.0))
                return s
            elif self.eltype == Element.TX:
                s.append(Element(eltype=Element.RY, constant=90.0))
                s.append(Element(elementIn=self, eltype=Element.TZ))
                s.append(Element(eltype=Element.RY, constant=-90.0))
                return s
            elif self.eltype == Element.TY:
                s.append(Element(eltype=Element.RX, constant=-90.0))
                s.append(Element(elementIn=self, eltype=Element.TZ))
                s.append(Element(eltype=Element.RX, constant=90.0))
                return s
            else:
                return None
        else:
            if self.eltype == Element.RY:
                s.append(Element(eltype=Element.RZ, constant=90.0))
                s.append(Element(elementIn=self, eltype=Element.RX))
                s.append(Element(eltype=Element.RZ, constant=-90.0))
                self.rules[8] += 1
                return s
            elif self.eltype == Element.TY:
                if prev.eltype == Element.RZ:
                    s.append(Element(eltype=Element.RZ, constant=90.0))
                    s.append(Element(elementIn=self, eltype=Element.TX))
                    s.append(Element(eltype=Element.RZ, constant=-90.0))
                    self.rules[6] += 1
                    return s
                else:
                    s.append(Element(eltype=Element.RX, constant=-90.0))
                    s.append(Element(elementIn=self, eltype=Element.TZ))
                    s.append(Element(eltype=Element.RX, constant=90.0))
                    self.rules[7] += 1
                    return s
            else:
                return None

    def substituteY(self, prev, next):
        s = list()

        if prev.isjoint() or self.isjoint():
            return None

        # note that if the rotation is -90 we must make the displacement -ve
        if prev.eltype == Element.RX and self.eltype == Element.TY:
            # RX.TY -> TZ.RX
            s.append(Element(elementIn=self, eltype=Element.TZ, constant=prev.constant))
            s.append(Element(elementIn=prev))
            self.rules[0] += 1
            return s
        elif prev.eltype == Element.RX and self.eltype == Element.TZ:
            # RX.TZ -> TY.RX
            s.append(Element(elementIn=self, eltype=Element.TY, constant=-prev.constant))
            s.append(Element(elementIn=prev))
            self.rules[2] += 1
            return s
        elif prev.eltype == Element.RY and self.eltype == Element.TX:
            # RY.TX -> TZ.RY
            s.append(Element(elementIn=self, eltype=Element.TZ, constant=-prev.constant))
            s.append(Element(elementIn=prev))
            self.rules[1] += 1
            return s
        elif prev.eltype == Element.RY and self.eltype == Element.TZ:
            # RY.TZ -> TX.RY
            s.append(Element(elementIn=self, eltype=Element.TX, constant=prev.constant))
            s.append(Element(elementIn=prev))
            self.rules[11] += 1
            return s
        elif prev.eltype == Element.TY and self.eltype == Element.RX:
            # TY.RX -> RX.TZ
            s.append(Element(elementIn=prev))
            s.append(Element(elementIn=self, eltype=Element.TZ, constant=-self.constant))
            self.rules[5] += 1
            # return s
            return None
        elif prev.eltype == Element.TX and self.eltype == Element.RZ:
            # TX.RZ -> RZ.TY
            s.append(Element(elementIn=prev))
            s.append(Element(elementIn=self, eltype=Element.TY, constant=self.constant))
            self.rules[5] += 1
            return s
        elif prev.eltype == Element.RY and self.eltype == Element.RX:
            # RY(Q).RX -> RX.RZ(-Q)
            s.append(Element(elementIn=prev))
            s.append(Element(elementIn=self, eltype=Element.RZ, constant=-1))
            self.rules[3] += 1
            return s
        elif prev.eltype == Element.RX and self.eltype == Element.RY:
            # RX.RY -> RZ.RX
            s.append(Element(elementIn=self, eltype=Element.RZ))
            s.append(Element(elementIn=prev))
            self.rules[4] += 1
            return s
        elif prev.eltype == Element.RZ and self.eltype == Element.RX:
            # RZ.RX -> RX.RY
            s.append(Element(elementIn=prev))
            s.append(Element(elementIn=self, eltype=Element.RY))
            # self.rules[10] += 1
            # return s
            return None
        return None

    # negate the arguments of the element
    def negate(self):

        self.constant = -self.constant

        if self.symconst:
            s = list(self.symconst)
            # if no leading sign character insert one (so we can flip it)
            if s[0] != "+" and s[0] != "-":
                s.insert(0, "+")
            for i in range(len(s)):
                if s[i] == "+":
                    s[i] = "-"
                elif s[i] == "-":
                    s[i] = "+"
                if s[0] == "+":
                    s.pop(0)
        s = "".join(s)

    '''
    Return a string representation of the parameters (argument)
    of the element, which can be a number, symbolic constant,
    or a joint variable.
    '''
    def argString(self):
        s = ""

        if self.eltype == Element.RX or Element.RY or Element.RZ or \
            Element.TX or Element.TY or Element.TZ:
            if self.var:
                s = self.var
            if self.symconst:
                if self.var:
                    if self.symconst[0] != "-":
                        s = s + "+"
                s = s + self.symconst
            # constants always displayed with a sign character
            if self.constant and self.constant != 0.0:
                if self.constant >= 0.0:
                    s = s + "+" + '{0:.3f}'.format(self.constant)
                else:
                    s = s + '{0:.3f}'.format(self.constant)
        elif self.eltype == Element.DH_STANDARD or Element.DH_MODIFIED:
            # theta, d, a, alpha
            # theta
            if self.prismatic == 0:
                # revolute joint
                s = s + self.var
                if self.offset >= 0:
                    s = s + "+" + '{0:.3f}'.format(self.offset)
                elif self.offset < 0:
                    s = s + '{0:.3f}'.format(self.offset)
            else:
                # prismatic joint
                s = s + '{0:.3f}'.format(self.theta)
            s = s + ", "

            # d
            if self.prismatic > 0:
                s = s + self.var
            else:
                s = s + self.D if self.D else s + "0"
            s = s + ", "

            # a
            s = s + self.A if self.A else s + "0"
            s = s + ", "

            # alpha
            s = s + '{0:.3f}'.format(self.alpha)
        else:
            raise ValueError("bad Element type")
        return s

    def toString(self):
        s = Element.typeName[self.eltype] + "("
        s = s + self.argString()
        s = s + ")"
        return s

class ElementList(list):

    def __str__(self):
        return self.toString()

    def factorize(self, dhWhich, verbose):
        nfactors = 0
        for i in range(len(self)):
            j = i
            jvars = match = 0
            for f in range(4):
                if j >= len(self):
                    break
                e = self[j]
                if f == 0 and verbose:
                    print("Starting at " + e)
                ee = e
                ee.factorMatch(dhWhich, f, verbose)
                e.factorMatch(dhWhich, f, verbose)
                if e.factorMatch(dhWhich, f, verbose):
                    j += 1     # move on to next element
                    match += 1
                    if e.isjoint():
                        jvars += 1
                    if jvars > 1:
                        break

            if match == 0 or jvars == 0:
                continue

            if verbose:
                print(" found subexpression " + match + " " + jvars)

            start = i
            end = j
            if jvars > 1:
                end -= 1

            dh = Element(eltype=dhWhich)

            for j in range(start, end):
                dh.add(self[i])
                self.pop(i)

            self.insert(i, dh)
            nfactors += 1
            if verbose:
                print(" result: " + dh)

        return nfactors

    '''
    Attempt to 'float' translational terms as far to the right as
    possible and across joint boundaries.
    '''
    def floatRight(self):
        j = 0
        nchanges = 0
        for i in range(len(self)-1):
            e = self[i]
            if e.isjoint():
                continue
            if not e.istrans():
                continue
            f = None
            crossed = False
            for jj in range(i+1, len(self)-1):
                f = self[jj]
                if f.istrans():
                    continue
                if f.isrot() and f.axis() == e.axis():
                    crossed = True
                    continue
                j = jj
                break
            if crossed and f:
                print("Float: " + e + " to " + f)
                self.pop(i)
                self.insert(j-1, e)
                nchanges += 1
                i -= 1

        return nchanges

    '''
    Swap adjacent terms according to inbuilt rules so as to achieve
    desired term ordering.
    '''
    def swap(self, dhWhich):
        total_changes = 0
        nchanges = 0

        while True:
            nchanges = 0

            for i in range(len(self)-1):
                e = self[i]
                if e.swap( self[i+1], dhWhich ):
                    self.pop(i)
                    self.insert(i+1, e)
                    nchanges += 1
            total_changes += nchanges
            if not nchanges:
                break
        return total_changes

    def substitutetoZ(self):
        replacement = list()
        nchanges = 0

        for i in range(len(self)):
            e = self[i]
            if not e.isjoint():
                continue
            replacement = e.substituteToZ()
            if replacement:
                # diagnostic string
                print("ReplaceToZ: " + e + " := ")
                for j in range(len(replacement)):
                    print(replacement[j])

                self.pop(i)
                for j in range(len(replacement)-1, -1, -1):
                    self.insert(i, replacement[j])
                i += len(replacement)-1
                nchanges += 1
        return nchanges

    '''
    substitute all non Z joint transforms according to rules.
    '''
    def substituteToZ2(self):
        replacement = list()
        nchanges = 0
        jointYet = False

        for i in range(len(self)):
            e = self[i]
            if e.isjoint():
                jointYet = True
                continue
            if i == 0 or not jointYet:  # leave initial const xform
                continue
            prev = self[i-1]
            replacement = e.substituteToZ(prev)
            if replacement:
                # diagnostic string
                print("ReplaceToZ2: " + e + " := ")
                for j in range(len(replacement)):
                    print("replacement[j]")

                self.pop(i)
                for j in range(len(replacement)-1, -1, -1):
                    self.insert(i, replacement[j])
                i += len(replacement)-1
                nchanges += 1
        return nchanges

    '''
    substitute transforms according to rules
    '''
    def substituteY(self):
        replacement = list()
        nchanges = 0
        jointYet = False

        for i in range(1, len(self)):
            e = self[i]
            if e.isjoint():
                jointYet = True
            if i == 0 or not jointYet:
                continue
            prev = self[i-1]
            if i+1 < len(self):
                next = self[i+1]
            else:
                next = None
            replacement = e.substituteY(prev, next)
            if replacement:
                # diagnostic string
                print("ReplaceY: " + prev + e + " := ")
                for j in range(len(replacement)):
                    print(replacement[j])

                self.pop(i)
                self.pop(i-1)
                for j in range(len(replacement)-1, -1, -1):
                    self.insert(i-1, replacement[j])
                i += len(replacement)-2
                nchanges += 1

        return nchanges

    '''
    merge adjacent transforms according to rules
    '''
    def merge(self):
        nchanges = 0

        for i in range(len(self)-1):
            e = self[i]
            e = e.merge(self[i+1])
            if e == self[i]:
                continue
            self.pop(i)
            self.pop(i)
            if e:
                self.insert(i, e)
            nchanges += 1

        return nchanges

    '''
    simplify expression. Cycle contiually around merging, substituting,
    and swapping until no more changes occur.
    '''
    def simplify(self):
        nchanges = 0
        nloops = 0

        '''
        simplify as much as possible, then substitue for all
        joint variables not in Z.
        '''
        self.merge()
        self.swap(Element.DH_STANDARD)
        self.merge()
        print(self)
        self.floatRight()
        self.merge()
        print("initial merge + swap")
        print(self)
        self.substitutetoZ()
        self.merge()
        print("joint vars to Z")
        print(self)
        print("0-------------------------------------------")
        while True:
            nchanges = 0

            nchanges += self.merge()
            nchanges += self.swap(Element.DH_STANDARD)
            nchanges += self.merge()
            nchanges += self.substituteY()
            nchanges += self.merge()
            print(self)
            print("1-------------------------------------------")
            if nchanges == 0:
                print("** deal with Ry/Ty")
                nchanges += self.substituteToZ2()
                nchanges += self.merge()
            if not (nchanges > 0 and nloops < 10):
                break
            else:
                nloops += 1

    def toString(self):
        s = ""

        for i in range(len(self)):
            s += str(self[i]) + ("." if i < len(self)-1 else "")

        return s

class DHFactor():

    def __init__(self, src):
        self.results = ElementList()
        try:
            results = self.parseString(src)
            print("In DHFactor, parseString is done")
        except ValueError:
            print("Value Error")
        if not self.isValid():
            print("DHFactor: error: Incomplete factorization, no DH equivalent found")

    def __str__(self):
        return self.toString()

    def toString(self):
        return self.results.toString()

    def angle(self, e):
        if isinstance(e, Element):
            return self.angle(e.constant)
        elif isinstance(e, int) or isinstance(e, float):
            if e == 0:
                return "0"
            elif e == 90.0:
                return "pi/2"
            elif e == -90.0:
                return "-pi/2"
            elif e == 180.0:
                return "pi"
            elif e == -180.0:
                return "-pi"
            else:
                raise ValueError("bad transform angle: " + e)

    '''
    replaces matlab method el2matlab()
    '''
    def el2spatialmath(self, start, end):
        xform = ""

        for i in range(start, end):
            e = self.results[0]

            if len(xform) > 0:
                xform += "*"

            if e.eltype == Element.RX: xform += "trotx(" + self.angle(e) + ")"
            elif e.eltype == Element.RY: xform += "troty(" + self.angle(e) + ")"
            elif e.eltype == Element.RZ: xform += "trotz(" + self.angle(e) + ")"
            elif e.eltype == Element.TX: xform += "transl(" + e.symconst + ",0,0)"
            elif e.eltype == Element.TY: xform += "transl(0," + e.symconst + ",0)"
            elif e.eltype == Element.TZ: xform += "transl(0,0," + e.symconst + ")"

        if len(xform) == 0:
            xform = "np.eye(4,4)"
        return xform

    '''
    Create a Toolbox legacy DH matrix. The column order is:
    
        theta d a alpha
    '''
    def dh(self):
        s = "["
        theta, d = str()

        for i in range(len(self.results)):
            e = self.results[i]
            if e.eltype == Element.DH_STANDARD:
                # build up the string: theta d a alpha
                if e.prismatic:
                    # prismatic joint
                    d = "0"
                    theta = self.angle(e.theta)
                else:
                    # revolute joint
                    theta = "0"
                    d = "0" if e.D is None else e.D

                s += "["
                s += theta
                s += ", "
                s += d
                s += ", "
                s += "0" if e.A is None else e.A
                s += ", "
                s += self.angle(e.alpha)
                s += ", " + e.prismatic
                s += "],"
        s += "]"
        return s

    '''
    Check the transform string is valid
    '''
    def isValid(self):
        iprev = -1

        for i in range(len(self.results)):
            e = self.results[i]
            if e.eltype == Element.DH_STANDARD:
                if iprev >= 0:
                    # we've seen a DH factor before
                    if i-iprev > 1:
                        # but it was too long ago, fail!
                        return False
                iprev = i   # note where we saw it
        return True

    def offset(self):
        s = "["

        for i in range(len(self.results)):
            e = self.results[i]
            if e.eltype == Element.DH_STANDARD:
                s += self.angle(e.offset) + " "

        s += "]"
        return s

    # return base transform string in spatial math form
    def base(self):
        for i in range(len(self.results)):
            e = self.results[i]

            if e.eltype == Element.DH_STANDARD or e.eltype == Element.DH_MODIFIED:
                return self.el2spatialmath(0, i)
        return "np.eye(4,4)"

    # return base transform string in spatial math form
    def tool(self):
        for i in range(len(self.results)-1, -1, -1):
            e = self.results[i]

            if e.eltype == Element.DH_STANDARD or e.eltype == Element.DH_MODIFIED:
                return self.el2spatialmath(i, len(self.results))
        return "np.eye(4,4)"

    # return rtb robot creation command
    def command(self, name):
        if self.isValid():
            return "DHRobot(L=" + self.dh() + ", name=" + name +\
                ", base=" + self.base() +\
                ", tool=" + self.tool() +\
                ", offset=" + self.offset() + ")"
        else:
            raise Exception("incompletely factored transform string")

    def parseFile(self, filename):

        file = open(filename)
        src = file.readline()

        return self.parseString(src)

    def parseString(self, buffer):
        l = ElementList()

        print("INIT: " + buffer)

        # each token is [R|T][x|y|z](arg)
        pattern = re.compile(r"([RT][xyz]\([^)]+\))")
        tokens = pattern.findall(buffer)
        for i in tokens:
            l.append(Element(stringIn=i))

        print("PARSED: " + str(l))

        l.simplify()
        print(l)

        l.factorize(Element.DH_STANDARD, 0)
        print(l)

        return l
