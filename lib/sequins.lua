S = {}
local function a(b)
    if type(b) == 'string' then
        local c = {}
        b:gsub('.', function(d)
            table.insert(c, d)
        end)
        return c
    end
    return b
end
function S.new(b)
    b = a(b)
    local e = {
        data = b,
        length = #b,
        ix = 1,
        qix = 1,
        n = 1,
        flw = {},
        fun = {}
    }
    return setmetatable(e, S)
end
local function f(e, g)
    return (g - 1) % e.length + 1
end
function S:is_sequins()
    return getmetatable(self) == S
end
function S:setdata(b)
    if S.is_sequins(b) then
        do
            local h = b.flw;
            local i = self.flw;
            for j, k in pairs(i) do
                if h[j] then
                    i[j].n = h[j].n
                else
                    i[j] = nil
                end
            end
            for j, k in pairs(h) do
                if not i[j] then
                    i[j] = {
                        ix = k.ix,
                        n = k.n
                    }
                end
            end
        end
        do
            local h = b.fun;
            local i = self.fun;
            if i[1] then
                i[1] = h[1]
                if h[2] then
                    for j, k in ipairs(h[2]) do
                        if S.is_sequins(k) and i[2] and S.is_sequins(i[2][j]) then
                            i[2][j]:settable(k)
                        else
                            i[2][j] = k
                        end
                    end
                end
            elseif h[1] then
                print 'new transformer'
                self.fun = b.fun
            end
        end
        b = b.data
    end
    b = a(b)
    for l = 1, #b do
        if S.is_sequins(b[l]) and S.is_sequins(self.data[l]) then
            self.data[l]:settable(b[l])
        else
            self.data[l] = b[l]
        end
    end
    self.data[#b + 1] = nil;
    self.length = #b;
    self.ix = f(self, self.ix)
end
function S:copy()
    if type(self) == 'table' then
        local m = {}
        for j, k in pairs(self) do
            m[j] = S.copy(k)
        end
        return setmetatable(m, getmetatable(self))
    else
        return self
    end
end
function S:peek()
    return self.data[self.ix]
end
function S:bake(n)
    n = n or #self;
    local b = {}
    for l = 1, n do
        b[l] = self()
    end
    return S.new(b)
end
local function o(b, p)
    p = p or S.next;
    if S.is_sequins(b) then
        return p(b)
    end
    return b
end
function S:func(p, ...)
    self.fun = {p, {...}}
    return self
end
local function q(e, k)
    if e.fun[1] then
        if #e.fun[2] > 0 then
            return e.fun[1](k, table.unpack(e.fun[2]))
        else
            return e.fun[1](k)
        end
    else
        return k
    end
end
S._fns = {
    add = function(n, r)
        return n + o(r)
    end,
    sub = function(n, r)
        return n - o(r)
    end,
    mul = function(n, r)
        return n * o(r)
    end,
    div = function(n, r)
        return n / o(r)
    end,
    mod = function(n, r)
        return n % o(r)
    end
}
S.__add = function(s, r)
    return S.func(s, S._fns.add, r)
end;
S.__sub = function(s, r)
    return S.func(s, S._fns.sub, r)
end;
S.__mul = function(s, r)
    return S.func(s, S._fns.mul, r)
end;
S.__div = function(s, r)
    return S.func(s, S._fns.div, r)
end;
S.__mod = function(s, r)
    return S.func(s, S._fns.mod, r)
end;
local function t(e)
    local u = f(e, e.qix or e.ix + o(e.n))
    local v, w = o(e.data[u])
    if w ~= 'again' then
        e.ix = u;
        e.qix = nil
    end
    if v == 'skip' or v == 'dead' then
        return S.next(e)
    end
    return v, w
end
S.flows = {
    every = function(x, n)
        return x.ix % n ~= 0
    end,
    times = function(x, n)
        return x.ix > n
    end,
    count = function(x, n)
        if x.ix < n then
            return 'again'
        else
            x.ix = 0
        end
    end
}
local function y(e, j)
    local x = e.flw[j]
    if x then
        x.ix = x.ix + 1;
        return S.flows[j](x, o(x.n))
    end
end
function S.next(e)
    if y(e, 'every') then
        return 'skip'
    end
    if y(e, 'times') then
        return 'dead'
    end
    local z = y(e, 'count')
    if z then
        local A = e.flw.every;
        if A then
            A.ix = A.ix - 1
        end
    end
    return q(e, t(e)), z
end
function S:step(n)
    self.n = n;
    return self
end
function S.flow(e, j, n)
    e.flw[j] = {
        n = n,
        ix = 0
    }
    return e
end
function S:every(n)
    return self:flow('every', n)
end
function S:count(n)
    return self:flow('count', n)
end
function S:times(n)
    return self:flow('times', n)
end
function S:all()
    return self:flow('count', #self)
end
function S:select(g)
    rawset(self, 'qix', g)
    return self
end
function S:reset()
    self:select(1)
    for B, k in ipairs(self.data) do
        o(k, S.reset)
    end
    for B, k in pairs(self.flw) do
        k.ix = 0;
        o(k.n, S.reset)
    end
    if self.fun[1] and #self.fun[2] > 0 then
        for B, k in pairs(self.fun[2]) do
            o(k, S.reset)
        end
    end
end
S.__call = function(self, ...)
    return self == S and S.new(...) or S.next(self)
end;
S.metaix = {
    settable = S.setdata,
    step = S.step,
    flow = S.flow,
    every = S.every,
    times = S.times,
    count = S.count,
    all = S.all,
    reset = S.reset,
    select = S.select,
    peek = S.peek,
    copy = S.copy,
    map = S.func,
    bake = S.bake
}
S.__index = function(self, g)
    if type(g) == 'number' then
        return self.data[g]
    else
        return S.metaix[g]
    end
end;
S.__newindex = function(self, g, k)
    if type(g) == 'number' then
        self.data[g] = k
    elseif g == 'n' then
        rawset(self, g, k)
    end
end;
S.__len = function(b)
    return b.length
end;
setmetatable(S, S)
