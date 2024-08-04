local function load_config(package)
    return function()
        require('plugins.' .. package)
    end
end

-- Uncomment the line below to load only the nexessary plugins
-- local plugins2 = require('plugins.otherPlugins').plugins2
-- local plugins2 = {}
local plugins = {
    ---- ======================== first priority  =======================
    {
        "folke/tokyonight.nvim",
        dependencies = {
            "shatur/neovim-ayu",
            'navarasu/onedark.nvim',
            "ellisonleao/gruvbox.nvim", --7/10
            "lunacookies/vim-colors-xcode",
            "catppuccin/nvim",
            "Mofiqul/dracula.nvim",

            "rebelot/kanagawa.nvim", --5/10
            'nyoom-engineering/oxocarbon.nvim',
            "bluz71/vim-moonfly-colors"
        },
        config = load_config('MainPlugins.colorscheme'),
        lazy = false,
        -- event = { 'BufReadPre', 'BufNewFile', 'VimEnter' },
        priority = 1000,
    },
    -- Telescope
    {
        'nvim-telescope/telescope.nvim',
        branch = '0.1.x',
        dependencies = {
            'nvim-lua/plenary.nvim',
            {
                'nvim-telescope/telescope-fzf-native.nvim',
                build = 'make',
            },
            'molecule-man/telescope-menufacture', --?? needs exploration
            "nvim-telescope/telescope-live-grep-args.nvim",
        },
        config = load_config('MainPlugins.telescope'),
        cmd = 'Telescope',
    },
    {
        'ThePrimeagen/harpoon',
        branch = 'harpoon2',
        config = load_config('MainPlugins.harpoon'),
        event = { 'BufReadPre', 'BufNewFile', 'VimEnter' },
        priority = 1000,
    },
    {
        'stevearc/oil.nvim',
        opts = {},
        -- optional dependencies
        dependencies = { "nvim-tree/nvim-web-devicons" },
        config = load_config('MainPlugins.oil'),
        event = { 'vimenter' },
    },
    {
        'echasnovski/mini.comment',
        version = '*',
        dependencies = 'JoosepAlviste/nvim-ts-context-commentstring',
        config = load_config('MainPlugins.comment'),
        event = { 'BufReadPre', 'BufNewFile' },
    },
    {
        'rcarriga/nvim-notify',
        config = load_config('MainPlugins.notify'),
        event = 'VeryLazy',
        cmd = 'Notifications',
    },
    {
        'nvim-lualine/lualine.nvim',
        config = load_config('MainPlugins.lualine'),
        event = { 'BufReadPre', 'BufNewFile' },
    },
    {
        'folke/which-key.nvim',
        config = load_config('MainPlugins.which-key'),
        event = 'VeryLazy',
    },

}

-- for _, plugin in ipairs(plugins2) do
--     table.insert(plugins, plugin)
-- end

local lsp_servers = {
    -- 'lua_ls',
    -- 'clangd',
    -- 'tsserver',
    -- 'html-lsp',
}

local ts_parsers = {
    -- 'javascript',
    -- 'lua',
    -- "c",
}

return {
    plugins = plugins,
    lsp_servers = lsp_servers,
    ts_parsers = ts_parsers,
}
