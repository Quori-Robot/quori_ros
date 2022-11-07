// Generate random number between the bounds
function random(lower, upper) {
    return Math.random() * (Math.max(lower, upper) - Math.min(lower, upper)) + Math.min(lower, upper);
}

// Generate random integer between the bounds
function randInt(lower, upper) {
    return Math.floor(random(lower, upper));
}

// Extract a random element from an array
function randomChoice(array) {
    return array[randInt(0, array.length)];
}

function coin() {
    return randInt(0, 2);
}

function remap(input, x1, x2, y1, y2) {
    return (input - x1) * ((y2 - y1) / (x2 - x1)) + y1;
}

function sum(list) {
    var total = 0;
    for (var i = 0; i < list.length; i++) {
        total += list[i];
    }
    return total;
}

function namespace(namespaceString) {
    var parts = namespaceString.split('.'),
        parent = window,
        currentPart = '';

    for(var i = 0, length = parts.length; i < length; i++) {
        currentPart = parts[i];
        parent[currentPart] = parent[currentPart] || {};
        parent = parent[currentPart];
    }

    return parent;
}