package org.firstinspires.ftc.teamcode.util;

import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Class to build strings formatted with HTML tags. Follows a builder format, so usage looks like this:
 * <p>{@code String message = new HtmlFormatter().bold().italic().textColor("blue").format("hi");}</p>
 * <p>Make sure that you set the telemetry display format:
 * {@code telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);}
 * </p>
 * <p>Note: not all tags are guaranteed to work with telemetry</p>
 */
@SuppressWarnings("unused")
public class HtmlFormatter implements Cloneable {

    protected Set<String> tags;
    protected Map<String, String> cssStyles;


    public HtmlFormatter() {
        tags = new HashSet<>();
        cssStyles = new HashMap<>();
    }

    /**
     * Toggle whether the text is bolded.
     * @return a copy of the current instance of {@link HtmlFormatter}
     */
    public HtmlFormatter bold() {
        if (!tags.remove("b")) tags.add("b");
        return clone();
    }

    /**
     * Toggle whether the text is italicized.
     * @return a copy of the current instance of {@link HtmlFormatter}
     */
    public HtmlFormatter italic() {
        if (!tags.remove("i")) tags.add("i");
        return clone();
    }

    /**
     * Toggle whether the text has a strikethrough.
     * @return a copy of the current instance of {@link HtmlFormatter}
     */
    public HtmlFormatter strikethrough() {
        if (!tags.remove("del")) tags.add("del");
        return clone();
    }

    /**
     * Toggle whether the text is underlined.
     * @return a copy of the current instance of {@link HtmlFormatter}
     */
    public HtmlFormatter underline() {
        if (!tags.remove("ins")) tags.add("ins");
        return clone();
    }

    /**
     * Toggle whether the text is a subscript.
     * @return a copy of the current instance of {@link HtmlFormatter}
     */
    public HtmlFormatter subscript() {
        if (!tags.remove("sub")) tags.add("sub");
        return clone();
    }

    /**
     * Toggle whether the text is a superscript.
     * @return a copy of the current instance of {@link HtmlFormatter}
     */
    public HtmlFormatter superscript() {
        if (!tags.remove("sup")) tags.add("sup");
        return clone();
    }

    /**
     * Toggle whether the text is small.
     * @return a copy of the current instance of {@link HtmlFormatter}
     */
    public HtmlFormatter small() {
        if (!tags.remove("small")) tags.add("small");
        return clone();
    }

    /**
     * Sets the color of the text. The color can be a color name (eg red, blue, MediumSeaGreen),
     * a hex value (eg #ff0000, #3b72d9), or other color value (eg rgb(100, 50, 20), hsl(20, 60, 100))
     *
     * @param color color of the text
     * @return a copy of the current instance of {@link HtmlFormatter}
     */
    public HtmlFormatter textColor(String color) {
        cssStyles.put("color", color);
        return clone();
    }

    /**
     * IMPORTANT NOTE: DOES NOT APPEAR TO WORK AND WILL BREAK OTHER HTML FUNCTIONS
     *
     * <p>Sets the color of the background, essentially highlighting the text. The color can be a color
     * name (eg red, blue, MediumSeaGreen), a hex value (eg #ff0000, #3b72d9), or other color value
     * (eg rgb(100, 50, 20), hsl(20, 60, 100))</p>
     *
     * @param color color of the background
     * @return a copy of the current instance of {@link HtmlFormatter}
     */
    public HtmlFormatter backgroundColor(String color) {
        cssStyles.put("background-color", color);
        return clone();
    }

    /**
     * Sets the font for the text. Add multiple fonts as backups, in case the font(s) before aren't
     * recognized and rendered.
     *
     * @param fonts font (plus backups) to use
     * @return a copy of the current instance of {@link HtmlFormatter}
     */
    public HtmlFormatter fontFamily(String... fonts) {
        StringBuilder str = new StringBuilder();
        for (String font : fonts)
            str.append(String.format("'%s', ", font));
        cssStyles.put("font-family", str.substring(0, str.length()-2));
        return clone();
    }

    public HtmlFormatter lineDecorationTypes(String... types) {
        StringBuilder str = new StringBuilder();
        for (String type : types)
            str.append(" ").append(type);
        cssStyles.put("text-decoration-line", str.substring(1));
        return clone();
    }

    public HtmlFormatter lineDecorationColor(String color) {
        cssStyles.put("text-decoration-color", color);
        return clone();
    }

    public HtmlFormatter lineDecorationStyle(String style) {
        cssStyles.put("text-decoration-color", style);
        return clone();
    }

    public HtmlFormatter lineDecorationThickness(String thickness) {
        cssStyles.put("text-decoration-color", thickness);
        return clone();
    }

    public HtmlFormatter textIndent(int pixels) {
        cssStyles.put("text-indent", pixels + "px");
        return clone();
    }

    public HtmlFormatter letterSpacing(int pixels) {
        cssStyles.put("letter-spacing", pixels + "px");
        return clone();
    }

    public HtmlFormatter lineHeight(double height) {
        cssStyles.put("line-height", height + "");
        return clone();
    }

    public HtmlFormatter wordSpacing(int pixels) {
        cssStyles.put("word-spacing", pixels + "px");
        return clone();
    }

    public HtmlFormatter whiteSpace(String type) {
        cssStyles.put("white-space", type);
        return clone();
    }

    /**
     * Create a shadow underneath the text.
     *
     * @param x horizontal offset of the shadow
     * @param y vertical offset of the shadow
     * @param blur radius of the blur of the shadow
     * @param color color of the shadow
     * @return a copy of the current instance of {@link HtmlFormatter}
     */
    public HtmlFormatter textShadow(int x, int y, int blur, String color) { // todo multi shadow
        cssStyles.put("text-shadow", String.format("%spx %spx %spx %s", x, y , blur, color));
        return clone();
    }


    public static String escapeHTML(String str) {
        return str.replaceAll("<","&lt")
                .replaceAll(">", "&gt;")
                .replaceAll(" ", "&nbsp;")
                .replaceAll("\"", "&quot")
                .replaceAll("'", "&apos;");
    }

    public String format(String text) {
        StringBuilder start = new StringBuilder();
        StringBuilder end = new StringBuilder();
        if (!cssStyles.isEmpty()) {
            start = new StringBuilder("<span style=\"");
            for (Map.Entry<String, String> entry: cssStyles.entrySet()) {
                start.append(String.format("%s:%s;", entry.getKey(), entry.getValue()));
            }
            start.append("\">");
            end = new StringBuilder("</span>");
        }
        for (String tag : tags) {
            start.append(String.format("<%s>", tag));
            end.insert(0, String.format("</%s>", tag));
        }
        String str = escapeHTML(text);
        return start + str + end;
    }

    public String format(Object obj) {
        return format(String.valueOf(obj));
    }

    @NotNull
    @Override
    public HtmlFormatter clone() {
        try {
            HtmlFormatter clone = (HtmlFormatter) super.clone();
            clone.tags = new HashSet<>(tags);
            clone.cssStyles = new HashMap<>(cssStyles);
            return clone;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}
