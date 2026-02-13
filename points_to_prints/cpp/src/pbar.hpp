#pragma once

#include <indicators/cursor_control.hpp>
#include <indicators/progress_bar.hpp>
#include <indicators/setting.hpp>

struct ProgressBarTotal {
    template <typename... Options>
    ProgressBarTotal(std::size_t total_, std::string prefix_text,
                     Options &&...options)
        : total(total_), progress(0),
          pbar{
              indicators::option::BarWidth{50},
              indicators::option::Start{"["},
              indicators::option::Fill{"="},
              indicators::option::Lead{">"},
              indicators::option::Remainder{" "},
              indicators::option::End{"]"},
              indicators::option::PrefixText{prefix_text},
              indicators::option::ForegroundColor{indicators::Color::cyan},
              indicators::option::ShowPercentage{true},
              indicators::option::FontStyles{std::vector<indicators::FontStyle>{
                  indicators::FontStyle::bold}},
              indicators::option::ShowElapsedTime{true},
              indicators::option::ShowRemainingTime{true},
              std::forward<Options>(options)...} {}

    // Delete copy operations - ProgressBar cannot be copied
    ProgressBarTotal(const ProgressBarTotal &) = delete;
    ProgressBarTotal &operator=(const ProgressBarTotal &) = delete;

    void start() { indicators::show_console_cursor(false); }

    void finish() { indicators::show_console_cursor(true); }

    void update(size_t increment) {
        progress += increment;
        uint8_t percentage =
            static_cast<uint8_t>((static_cast<double>(progress) / total) * 100);
        if (progress == total) {
            pbar.set_progress(100);
        } else if (percentage > last_percentage) {
            last_percentage = percentage;
            pbar.set_progress(percentage);
        }
    }

  private:
    const size_t total;
    size_t progress;
    uint8_t last_percentage = 0;
    indicators::ProgressBar pbar;
};
