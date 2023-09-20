#pragma once
#include <cxxabi.h>

#include <iostream>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>
namespace ctl {
    /*
    #define hr_out  out(RED,HL)
    #define r_out   out(RED)
    #define hg_out  out(GREEN,HL)
    #define g_out   out(GREEN)
    #define hy_out  out(YELLOW,HL)
    #define y_out   out(YELLOW)
    #define hbl_out out(BLUE,HL)
    #define bl_out  out(BLUE)
    #define hp_out out(PURPLE,HL)
    #define p_out  out(PURPLE)
    #define hdg_out out(DGREEN,HL)
    #define dg_out  out(DGREEN)
    */
    template <typename T>
    std::ostream &operator<<(std::ostream &output, const std::vector<T> &vec) {
        for (auto &&item : vec) {
            output << item << " ";
        }

        return output;
    }
    static uint64_t record_points_num = 0;
#define CTL_AUTOTP(TAG)                                                               \
    ctl::hp_out() << "id: " << ctl::record_points_num++ << " [" << __FILE__ << "] : " \
                  << "(" << __LINE__ << ")"                                           \
                  << " tag: " << TAG << ctl::endl
#define CTL_TITLE_EXIT(INFO)                         \
    ctl::hr_out() << " [" << __FILE__ << "] : "      \
                  << "(" << __LINE__ << ")"          \
                  << " exit: " << INFO << ctl::endl; \
    exit(0)
    enum COLOR { BLACK = 0, RED, GREEN, YELLOW, BLUE, PURPLE, DGREEN, WHITE, DEFAULT, CRUNUSED };
    enum CTRL {
        CLEAR = 0,
        HL,  // Heigh Light
        DL,  // down line
        BK,  // Blink
        CLUNUSED
    };
    const int offset_front_color = 30;
    const int offset_back_color = 40;
    const std::string clear = "\033[0m";
    const std::string endl = "\033[0m\n";
    static std::string generate_ansi(enum COLOR front_color = CRUNUSED, enum CTRL ctrl = CLUNUSED,
                                     enum COLOR back_color = CRUNUSED) {
        std::string generate_ansi;
        if (ctrl != CLUNUSED) {
            generate_ansi += std::to_string(ctrl);
            generate_ansi += ";";
        }
        if (front_color != CRUNUSED) {
            generate_ansi += std::to_string(front_color + offset_front_color);
            generate_ansi += ";";
        }
        if (back_color != CRUNUSED) {
            generate_ansi += std::to_string(back_color + offset_back_color);
            generate_ansi += ";";
        }
        if (generate_ansi.empty()) {
            return std::string();
        } else {
            generate_ansi.pop_back();
            return "\033[" + generate_ansi + "m";
        }
    }

    static std::ostream &out(enum COLOR front_color = CRUNUSED, enum CTRL ctrl = CLUNUSED,
                             enum COLOR back_color = CRUNUSED) {
        std::string generate_ansi;
        if (ctrl != CLUNUSED) {
            generate_ansi += std::to_string(ctrl);
            generate_ansi += ";";
        }
        if (front_color != CRUNUSED) {
            generate_ansi += std::to_string(front_color + offset_front_color);
            generate_ansi += ";";
        }
        if (back_color != CRUNUSED) {
            generate_ansi += std::to_string(back_color + offset_back_color);
            generate_ansi += ";";
        }
        if (generate_ansi.empty()) {
            return std::cout;
        } else {
            generate_ansi.pop_back();
            return std::cout << "\033[" << generate_ansi << "m";
        }
    }
    inline std::ostream &hr_out() { return out(RED, HL); }
    inline std::ostream &r_out() { return out(RED); }

    inline std::ostream &hg_out() { return out(GREEN, HL); }
    inline std::ostream &g_out() { return out(GREEN); }

    inline std::ostream &hy_out() { return out(YELLOW, HL); }
    inline std::ostream &y_out() { return out(YELLOW); }

    inline std::ostream &hbl_out() { return out(BLUE, HL); }
    inline std::ostream &bl_out() { return out(BLUE); }

    inline std::ostream &hp_out() { return out(PURPLE, HL); }
    inline std::ostream &p_out() { return out(PURPLE); }

    inline std::ostream &hdg_out() { return out(DGREEN, HL); }
    inline std::ostream &dg_out() { return out(DGREEN); }
    ///. table
    static int status;
    struct var_info {
        std::string var_name;
        std::string var_type;
        std::string var_val;
    };
    struct color_table_scheme {
        enum COLOR front_color = CRUNUSED;
        enum COLOR back_color = CRUNUSED;
        enum CTRL ctl = CLUNUSED;
    };

#define VAR_NAME(VAR) abi::__cxa_demangle(typeid(VAR).name(), 0, 0, &ctl::status)
#define CTL_TABLE_ADD_VAR(TABLE, VAR) TABLE->add_item(#VAR, VAR_NAME(VAR), VAR)

    class value_v_buff : private std::streambuf, public std::ostream {
       public:
        value_v_buff() : std::ostream(this) {}
        void clear() { buff_.clear(); }
        std::string get_buff() { return buff_; }

       private:
        std::string buff_;
        int overflow(int c) override {
            foo(c);
            return 0;
        }
        void foo(char c) {
            if (c == '\n') {
                c = '|';
            }

            buff_ += c;
        }
    };

    class table_out {
       public:
        table_out(std::string table_n = "default table", int padding_ = 2) {
            padding = padding_;
            table_name = table_n;
            clear();
            line_color.front_color = BLUE;
            title_color.ctl = HL;
            title_color.front_color = BLUE;
            head_color.front_color = PURPLE;
            head_color.ctl = HL;
        }
        template <typename T>
        void add_item(std::string value_n, std::string value_t, T t) {
            var_info vi;
            vi.var_name = value_n;
            if (private_abbrev.count(value_t)) {
                vi.var_type = private_abbrev[value_t];
            } else if (default_abbrev.count(value_t)) {
                vi.var_type = default_abbrev[value_t];
            } else {
                vi.var_type = value_t;
            }

            vvb.clear();
            vvb << t;
            vi.var_val = vvb.get_buff();
            var_infos.push_back(vi);
        };
        void clear() {
            var_infos.clear();
            ///.可以写成const
            var_info vi;
            vi.var_name = "Variant:";
            vi.var_type = "Type:";
            vi.var_val = "Value:";
            var_infos.push_back(vi);
        }
        void make_table_and_out() {
            if (var_infos.empty()) {
                std::cout << "empty table!!!" << std::endl;
                return;
            }
            ///. 确定尺寸
            std::vector<size_t> width(3, 0);
            for (auto &&vi : var_infos) {
                width[0] = std::max(width[0], vi.var_name.size());
                width[1] = std::max(width[1], vi.var_type.size());
                width[2] = std::max(width[2], vi.var_val.size());
            }
            int width_size = width[0] + width[1] + width[2] + padding * 6 + 4;
            ///. 绘制横线行
            std::string tline;
            std::string midline;
            tline += '+';
            tline += std::string(width_size - 2, '-');
            tline += "+";
            midline = tline;
            ///. 表头
            std::string padding_txt(padding, ' ');
            std::string table_txt;

            table_txt +=
                generate_ansi(line_color.front_color, line_color.ctl, line_color.back_color);
            table_txt += tline;
            table_txt += ctl::endl;

            std::string vline =
                generate_ansi(line_color.front_color, line_color.ctl, line_color.back_color) +
                "|\033[0m";

            std::string tmp;
            tmp = vline + padding_txt +
                  generate_ansi(title_color.front_color, title_color.ctl, title_color.back_color) +
                  table_name + "\033[0m";
            tmp += std::string(width_size - padding - table_name.size() - 2, ' ');
            tmp += vline + "\n";
            table_txt += tmp;
            midline[2 * padding + width[0] + 1] = '+';
            midline[4 * padding + width[0] + 2 + width[1]] = '+';
            midline = generate_ansi(BLUE) + midline + ctl::endl;
            table_txt += midline;
            for (size_t i = 0; i < var_infos.size(); i++) {
                color_table_scheme now_scheme = context_color;
                if (i == 0) {
                    now_scheme = head_color;
                }

                auto &vi = var_infos[i];
                tmp.clear();
                tmp = vline + padding_txt +
                      generate_ansi(now_scheme.front_color, now_scheme.ctl, now_scheme.back_color) +
                      vi.var_name + "\033[0m";
                tmp += std::string(width[0] - vi.var_name.size(), ' ');
                tmp +=
                    padding_txt + vline + padding_txt +
                    generate_ansi(now_scheme.front_color, now_scheme.ctl, now_scheme.back_color) +
                    vi.var_type + "\033[0m";
                tmp += std::string(width[1] - vi.var_type.size(), ' ');
                tmp +=
                    padding_txt + vline + padding_txt +
                    generate_ansi(now_scheme.front_color, now_scheme.ctl, now_scheme.back_color) +
                    vi.var_val + "\033[0m";
                tmp += std::string(width[2] - vi.var_val.size(), ' ');
                tmp += padding_txt + vline + "\n";
                table_txt += tmp;
                // if (i==var_infos.size()-1)
                // {
                //     table_txt+=tline;
                // }
                // else{
                //     table_txt+=midline;
                // }
                table_txt += midline;
            }

            std::cout << table_txt << std::endl;
        }

        color_table_scheme line_color;
        color_table_scheme title_color;
        color_table_scheme head_color;
        color_table_scheme context_color;
        int padding;
        std::string table_name;
        std::unordered_map<std::string, std::string> private_abbrev;

       private:
        value_v_buff vvb;
        std::vector<var_info> var_infos;
        std::unordered_map<std::string, std::string> default_abbrev = {
            {"std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >",
             "string"},
            {"std::vector<double, std::allocator<double> >", "vector<double>"},
            {"Eigen::Matrix<double, 3, 3, 0, 3, 3>", "Eigen::Matrix3d"}};
    };

}  // namespace ctl
