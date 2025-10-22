package params

import (
	"fmt"
	"os"
	"strconv"
	"strings"
)

const envVarPrefix = "RCL_PARAM_"

// ApplyEnvVars scans the environment for RCL_PARAM_* variables and applies them
// to the parameter manager. Environment variables follow the pattern:
//
//	RCL_PARAM_<param_name>=<value>
//
// For example:
//
//	RCL_PARAM_camera.fps=30
//	RCL_PARAM_debug_mode=true
//	RCL_PARAM_tags=[tag1,tag2,tag3]
//
// Type inference:
//   - If parameter already declared: uses existing parameter type
//   - If not declared: infers type from string value
//   - "true"/"false" → bool
//   - Numeric with decimal point → double
//   - Numeric without decimal → int64
//   - Array syntax [a,b,c] → array type (inferred from elements)
//   - Otherwise → string
//
// Priority order (recommended usage):
//  1. Defaults (declared in code)
//  2. Environment variables (ApplyEnvVars)
//  3. YAML files (LoadYAML)
//  4. CLI overrides (ApplyOverrides) - highest priority
//
// Usage:
//
//	mgr, _ := params.NewManager(node)
//	_ = params.ApplyEnvVars(mgr)  // Apply env vars
//	_ = params.LoadYAML(mgr, nodeName, yamlPath)  // Load YAML (overrides env vars)
//	_ = params.ApplyOverrides(mgr, nodeName, rclArgs)  // CLI overrides (highest priority)
func ApplyEnvVars(m *Manager) error {
	envParams := getParamEnvVars()
	if len(envParams) == 0 {
		// No RCL_PARAM_* environment variables found
		return nil
	}

	for name, valStr := range envParams {
		// Check if parameter already exists
		existing, exists := m.Get(name)

		var val Value
		var err error

		if exists {
			// Use existing parameter type
			val, err = parseEnvVarValue(valStr, &existing)
		} else {
			// Infer type from string value
			val, err = parseEnvVarValue(valStr, nil)
		}

		if err != nil {
			return fmt.Errorf("failed to parse env var RCL_PARAM_%s: %w", name, err)
		}

		// If parameter exists, set it. Otherwise declare it.
		if exists {
			if _, err := m.SetValue(name, val); err != nil {
				return fmt.Errorf("failed to set env var for %s: %w", name, err)
			}
		} else {
			if _, err := m.Declare(name, val, Descriptor{
				Description: fmt.Sprintf("Set from environment variable RCL_PARAM_%s", name),
			}); err != nil {
				return fmt.Errorf("failed to declare env var for %s: %w", name, err)
			}
		}
	}

	return nil
}

// getParamEnvVars scans the environment for RCL_PARAM_* variables and returns
// a map of parameter names to values.
func getParamEnvVars() map[string]string {
	result := make(map[string]string)
	environ := os.Environ()

	for _, env := range environ {
		// Split on first '=' to get key=value
		parts := strings.SplitN(env, "=", 2)
		if len(parts) != 2 {
			continue
		}

		key := parts[0]
		value := parts[1]

		// Check if key starts with RCL_PARAM_
		if strings.HasPrefix(key, envVarPrefix) {
			// Extract parameter name (everything after prefix)
			paramName := strings.TrimPrefix(key, envVarPrefix)
			if paramName != "" {
				result[paramName] = value
			}
		}
	}

	return result
}

// parseEnvVarValue converts a string value to a Value, using the existing
// parameter's type if provided, otherwise inferring from the string.
func parseEnvVarValue(valStr string, existingParam *Parameter) (Value, error) {
	valStr = strings.TrimSpace(valStr)

	// If we have an existing parameter, use its type
	if existingParam != nil {
		return parseValueWithType(valStr, existingParam.Value.Kind)
	}

	// Otherwise, infer type from string
	return inferValueType(valStr)
}

// parseValueWithType parses a string value into the specified type.
func parseValueWithType(valStr string, kind uint8) (Value, error) {
	switch kind {
	case KindBool:
		return parseBool(valStr)
	case KindInt64:
		return parseInt64(valStr)
	case KindDouble:
		return parseDouble(valStr)
	case KindString:
		return Value{Kind: KindString, Str: valStr}, nil
	case KindBytes:
		return Value{Kind: KindBytes, Bytes: []byte(valStr)}, nil
	case KindBoolArray:
		return parseBoolArray(valStr)
	case KindInt64Array:
		return parseInt64Array(valStr)
	case KindDoubleArray:
		return parseDoubleArray(valStr)
	case KindStringArray:
		return parseStringArray(valStr)
	default:
		return Value{}, fmt.Errorf("unsupported parameter type: %d", kind)
	}
}

// inferValueType infers the Value type from a string representation.
func inferValueType(valStr string) (Value, error) {
	// Check for array syntax first
	if strings.HasPrefix(valStr, "[") && strings.HasSuffix(valStr, "]") {
		return inferArrayType(valStr)
	}

	// Check for boolean
	lower := strings.ToLower(valStr)
	if lower == "true" || lower == "false" {
		return parseBool(valStr)
	}

	// Try to parse as number
	if strings.Contains(valStr, ".") {
		// Has decimal point → try double
		if val, err := parseDouble(valStr); err == nil {
			return val, nil
		}
	} else {
		// No decimal point → try int64
		if val, err := parseInt64(valStr); err == nil {
			return val, nil
		}
	}

	// Default to string
	return Value{Kind: KindString, Str: valStr}, nil
}

// inferArrayType infers and parses an array value from string.
func inferArrayType(valStr string) (Value, error) {
	// Strip brackets
	inner := strings.TrimPrefix(valStr, "[")
	inner = strings.TrimSuffix(inner, "]")
	inner = strings.TrimSpace(inner)

	if inner == "" {
		// Empty array - default to string array
		return Value{Kind: KindStringArray, Strs: []string{}}, nil
	}

	// Split by comma
	elements := strings.Split(inner, ",")
	for i := range elements {
		elements[i] = strings.TrimSpace(elements[i])
	}

	if len(elements) == 0 {
		return Value{Kind: KindStringArray, Strs: []string{}}, nil
	}

	// Infer type from first element
	first := elements[0]
	lower := strings.ToLower(first)

	if lower == "true" || lower == "false" {
		// Bool array
		return parseBoolArray(valStr)
	}

	// Try numeric
	if strings.Contains(first, ".") {
		// Try double array
		if val, err := parseDoubleArray(valStr); err == nil {
			return val, nil
		}
	} else {
		// Try int array
		if val, err := parseInt64Array(valStr); err == nil {
			return val, nil
		}
	}

	// Default to string array
	return parseStringArray(valStr)
}

// Scalar parsers

func parseBool(valStr string) (Value, error) {
	b, err := strconv.ParseBool(valStr)
	if err != nil {
		return Value{}, err
	}
	return Value{Kind: KindBool, Bool: b}, nil
}

func parseInt64(valStr string) (Value, error) {
	i, err := strconv.ParseInt(valStr, 10, 64)
	if err != nil {
		return Value{}, err
	}
	return Value{Kind: KindInt64, Int64: i}, nil
}

func parseDouble(valStr string) (Value, error) {
	f, err := strconv.ParseFloat(valStr, 64)
	if err != nil {
		return Value{}, err
	}
	return Value{Kind: KindDouble, Double: f}, nil
}

// Array parsers

func parseBoolArray(valStr string) (Value, error) {
	elements, err := extractArrayElements(valStr)
	if err != nil {
		return Value{}, err
	}

	bools := make([]bool, len(elements))
	for i, elem := range elements {
		b, err := strconv.ParseBool(elem)
		if err != nil {
			return Value{}, fmt.Errorf("element %d: %w", i, err)
		}
		bools[i] = b
	}
	return Value{Kind: KindBoolArray, Bools: bools}, nil
}

func parseInt64Array(valStr string) (Value, error) {
	elements, err := extractArrayElements(valStr)
	if err != nil {
		return Value{}, err
	}

	ints := make([]int64, len(elements))
	for i, elem := range elements {
		n, err := strconv.ParseInt(elem, 10, 64)
		if err != nil {
			return Value{}, fmt.Errorf("element %d: %w", i, err)
		}
		ints[i] = n
	}
	return Value{Kind: KindInt64Array, Int64s: ints}, nil
}

func parseDoubleArray(valStr string) (Value, error) {
	elements, err := extractArrayElements(valStr)
	if err != nil {
		return Value{}, err
	}

	doubles := make([]float64, len(elements))
	for i, elem := range elements {
		f, err := strconv.ParseFloat(elem, 64)
		if err != nil {
			return Value{}, fmt.Errorf("element %d: %w", i, err)
		}
		doubles[i] = f
	}
	return Value{Kind: KindDoubleArray, Doubles: doubles}, nil
}

func parseStringArray(valStr string) (Value, error) {
	elements, err := extractArrayElements(valStr)
	if err != nil {
		return Value{}, err
	}

	// Remove quotes from string elements if present
	strs := make([]string, len(elements))
	for i, elem := range elements {
		strs[i] = strings.Trim(elem, `"'`)
	}
	return Value{Kind: KindStringArray, Strs: strs}, nil
}

// extractArrayElements splits an array string "[a,b,c]" into ["a", "b", "c"].
func extractArrayElements(valStr string) ([]string, error) {
	valStr = strings.TrimSpace(valStr)
	if !strings.HasPrefix(valStr, "[") || !strings.HasSuffix(valStr, "]") {
		return nil, fmt.Errorf("not an array (missing brackets): %s", valStr)
	}

	inner := strings.TrimPrefix(valStr, "[")
	inner = strings.TrimSuffix(inner, "]")
	inner = strings.TrimSpace(inner)

	if inner == "" {
		return []string{}, nil
	}

	elements := strings.Split(inner, ",")
	for i := range elements {
		elements[i] = strings.TrimSpace(elements[i])
	}

	return elements, nil
}
