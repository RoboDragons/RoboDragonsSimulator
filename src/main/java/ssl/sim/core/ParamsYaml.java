package ssl.sim.core;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import ssl.sim.ball.BallContacts;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.math.BigDecimal;
import java.util.Map;
import java.util.Set;

/** Generic commented-YAML writer / reader (SnakeYAML) for a class of public parameter fields. */
final class ParamsYaml {
    private ParamsYaml() {}

    static String toYaml(Object obj, Map<String, String> doc, Set<String> excluded, String header) {
        return toYaml(obj, doc, excluded, header, Map.of());
    }

    /** {@code scale}: field name -> factor from the internal unit to the unit written in the file (e.g. rad -> deg). */
    static String toYaml(Object obj, Map<String, String> doc, Set<String> excluded, String header, Map<String, Double> scale) {
        StringBuilder sb = new StringBuilder(header);
        try {
            for (Field f : obj.getClass().getFields()) {
                if (Modifier.isStatic(f.getModifiers()) || excluded.contains(f.getName())) continue;
                sb.append("\n");
                String d = doc.get(f.getName());
                sb.append("# ").append(d != null ? d : f.getName()).append("\n");
                Object v = f.get(obj);
                if (v instanceof BallContacts.Material m) {
                    sb.append(f.getName()).append(":\n");
                    sb.append("  stiffness: ").append(num(m.stiffness())).append("\n");
                    sb.append("  dampingRatio: ").append(num(m.dampingRatio())).append("\n");
                    sb.append("  mu: ").append(num(m.mu())).append("\n");
                } else if (v instanceof java.util.List<?> list && !list.isEmpty()) {
                    sb.append(f.getName()).append(":\n");
                    for (Object e : list) {
                        String lead = "  - ";
                        for (Field ef : e.getClass().getFields()) {
                            if (Modifier.isStatic(ef.getModifiers())) continue;
                            sb.append(lead).append(ef.getName()).append(": ").append(scalar(ef.get(e))).append("\n");
                            lead = "    ";
                        }
                    }
                } else if (v instanceof java.util.List<?>) {
                    sb.append(f.getName()).append(": []\n");
                } else {
                    sb.append(f.getName()).append(": ").append(scalar(scaled(v, scale.get(f.getName())))).append("\n");
                }
            }
        } catch (IllegalAccessException e) {
            throw new IllegalStateException(e);
        }
        return sb.toString();
    }

    private static Object scaled(Object v, Double factor) {
        if (factor == null) return v;
        if (v instanceof Double d) return d * factor;
        if (v instanceof double[] a) {
            double[] r = new double[a.length];
            for (int i = 0; i < a.length; i++) r[i] = a[i] * factor;
            return r;
        }
        return v;
    }

    private static JsonElement unscaled(JsonElement e, double factor) {
        if (e.isJsonPrimitive() && e.getAsJsonPrimitive().isNumber()) return new com.google.gson.JsonPrimitive(e.getAsDouble() / factor);
        if (e.isJsonArray()) {
            com.google.gson.JsonArray r = new com.google.gson.JsonArray();
            for (JsonElement x : e.getAsJsonArray()) r.add(unscaled(x, factor));
            return r;
        }
        return e;
    }

    private static String scalar(Object v) {
        if (v instanceof Double d) return num(d);
        if (v instanceof double[] a) {
            StringBuilder s = new StringBuilder("[");
            for (int i = 0; i < a.length; i++) s.append(i > 0 ? ", " : "").append(num(a[i]));
            return s.append(']').toString();
        }
        return String.valueOf(v); // int, boolean, String
    }

    /** Plain decimal notation (YAML 1.1 does not read "1.0E7" as a number). */
    private static String num(double d) {
        String s = new BigDecimal(Double.toString(d)).round(new java.math.MathContext(15)).stripTrailingZeros().toPlainString();
        return s.contains(".") ? s : s + ".0";
    }

    /** Parses YAML and applies the entries present on top of {@code target}; unknown keys are an error. */
    @SuppressWarnings("unchecked")
    static <T> void apply(String yaml, T target, Class<T> type, String what) {
        apply(yaml, target, type, what, Map.of());
    }

    @SuppressWarnings("unchecked")
    static <T> void apply(String yaml, T target, Class<T> type, String what, Map<String, Double> scale) {
        Object loaded = new org.yaml.snakeyaml.Yaml(new org.yaml.snakeyaml.constructor.SafeConstructor(
                new org.yaml.snakeyaml.LoaderOptions())).load(yaml);
        if (loaded == null) return;
        if (!(loaded instanceof Map)) throw new IllegalArgumentException(what + ": top level must be a mapping");
        Gson gson = new GsonBuilder().serializeNulls()
                .registerTypeAdapter(type, (com.google.gson.InstanceCreator<T>) t -> target)
                .create();
        JsonObject overlay = gson.toJsonTree(loaded).getAsJsonObject();
        for (String key : overlay.keySet()) {
            try {
                type.getField(key);
            } catch (NoSuchFieldException e) {
                throw new IllegalArgumentException(what + ": unknown parameter '" + key + "'");
            }
        }
        for (var s : scale.entrySet()) if (overlay.has(s.getKey())) overlay.add(s.getKey(), unscaled(overlay.get(s.getKey()), s.getValue()));
        JsonObject merged = gson.toJsonTree(target).getAsJsonObject();
        merge(merged, overlay);
        gson.fromJson(merged, type);
    }

    /** Recursive merge so that a partly given material keeps its other values. */
    private static void merge(JsonObject base, JsonObject over) {
        for (var e : over.entrySet()) {
            JsonElement b = base.get(e.getKey());
            if (b != null && b.isJsonObject() && e.getValue().isJsonObject()) merge(b.getAsJsonObject(), e.getValue().getAsJsonObject());
            else base.add(e.getKey(), e.getValue());
        }
    }
}
