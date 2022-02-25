package org.firstinspires.ftc.teamcode.util;

import java.util.*;

/**
 * Usage looks like this:
 * <p>
 * {@code String message = new HtmlFormatter("hi").bold().italic().textColor("blue").toString();}
 * </p>
 */
@SuppressWarnings("unused")
public class HtmlFormatter {

    protected String string;
    protected Set<String> tags;
    protected Map<String, String> cssStyles;

    public HtmlFormatter(String string) {
        this.string = string;
        tags = new HashSet<>();
        cssStyles = new HashMap<>();
    }

    /**
     * Toggle whether the text is bolded.
     * @return the instance of {@link HtmlFormatter}
     */
    public HtmlFormatter bold() {
        if (!tags.remove("b")) tags.add("b");
        return this;
    }

    /**
     * Toggle whether the text is italicized.
     * @return the instance of {@link HtmlFormatter}
     */
    public HtmlFormatter italic() {
        if (!tags.remove("i")) tags.add("i");
        return this;
    }

    /**
     * Toggle whether the text has a strikethrough.
     * @return the instance of {@link HtmlFormatter}
     */
    public HtmlFormatter strikethrough() {
        if (!tags.remove("del")) tags.add("del");
        return this;
    }

    /**
     * Toggle whether the text is underlined.
     * @return the instance of {@link HtmlFormatter}
     */
    public HtmlFormatter underline() {
        if (!tags.remove("ins")) tags.add("ins");
        return this;
    }

    /**
     * Toggle whether the text is a subscript.
     * @return the instance of {@link HtmlFormatter}
     */
    public HtmlFormatter subscript() {
        if (!tags.remove("sub")) tags.add("sub");
        return this;
    }

    /**
     * Toggle whether the text is a superscript.
     * @return the instance of {@link HtmlFormatter}
     */
    public HtmlFormatter superscript() {
        if (!tags.remove("sup")) tags.add("sup");
        return this;
    }

    /**
     * Toggle whether the text is small.
     * @return the instance of {@link HtmlFormatter}
     */
    public HtmlFormatter small() {
        if (!tags.remove("small")) tags.add("small");
        return this;
    }

    /**
     * Sets the color of the text. The color can be a color name (eg red, blue, MediumSeaGreen),
     * a hex value (eg #ff0000, #3b72d9), or other color value (eg rgb(100, 50, 20), hsl(20, 60, 100))
     *
     * @param color color of the text
     * @return the instance of {@link HtmlFormatter}
     */
    public HtmlFormatter textColor(String color) {
        cssStyles.put("color", color);
        return this;
    }

    /**
     * Sets the color of the background, essentially highlighting the text. The color can be a color
     * name (eg red, blue, MediumSeaGreen), a hex value (eg #ff0000, #3b72d9), or other color value
     * (eg rgb(100, 50, 20), hsl(20, 60, 100))
     *
     * @param color color of the background
     * @return the instance of {@link HtmlFormatter}
     */
    public HtmlFormatter backgroundColor(String color) {
        cssStyles.put("background-color", color);
        return this;
    }

    /**
     * Sets the font for the text. Add multiple fonts as backups, in case the font(s) before aren't
     * recognized and rendered.
     *
     * @param fonts font (plus backups) to use
     * @return the instance of {@link HtmlFormatter}
     */
    public HtmlFormatter fontFamily(String... fonts) {
        StringBuilder str = new StringBuilder();
        for (String font : fonts)
            str.append(String.format("'%s', ", font));
        cssStyles.put("font-family", str.substring(0, str.length()-2));
        return this;
    }

    public HtmlFormatter lineDecorationTypes(String... types) {
        String str = "";
        for (String type : types)
            str += " " + type;
        cssStyles.put("text-decoration-line", str.substring(1));
        return this;
    }

    public HtmlFormatter lineDecorationColor(String color) {
        cssStyles.put("text-decoration-color", color);
        return this;
    }

    public HtmlFormatter lineDecorationStyle(String style) {
        cssStyles.put("text-decoration-color", style);
        return this;
    }

    public HtmlFormatter lineDecorationThickness(String thickness) {
        cssStyles.put("text-decoration-color", thickness);
        return this;
    }

    public HtmlFormatter textIndent(int pixels) {
        cssStyles.put("text-indent", pixels + "px");
        return this;
    }

    public HtmlFormatter letterSpacing(int pixels) {
        cssStyles.put("letter-spacing", pixels + "px");
        return this;
    }

    public HtmlFormatter lineHeight(double height) {
        cssStyles.put("line-height", height + "");
        return this;
    }

    public HtmlFormatter wordSpacing(int pixels) {
        cssStyles.put("word-spacing", pixels + "px");
        return this;
    }

    public HtmlFormatter whiteSpace(String type) {
        cssStyles.put("white-space", type);
        return this;
    }

    /**
     * Create a shadow underneath the text.
     *
     * @param x horizontal offset of the shadow
     * @param y vertical offset of the shadow
     * @param blur radius of the blur of the shadow
     * @param color color of the shadow
     * @return the instance of {@link HtmlFormatter}
     */
    public HtmlFormatter textShadow(int x, int y, int blur, String color) { // todo multi shadow
        cssStyles.put("text-shadow", String.format("%spx %spx %spx %s", x, y , blur, color));
        return this;
    }


    private static String escapeHTML(String str) {
        return str.replaceAll("<","&lt")
                .replaceAll(">", "&gt;")
                .replaceAll(" ", "&nbsp;")
                .replaceAll("\"", "&quot")
                .replaceAll("'", "&apos;");
    }

    @Override
    public String toString() {
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
        String str = escapeHTML(string);
        return start + str + end;
    }
}
